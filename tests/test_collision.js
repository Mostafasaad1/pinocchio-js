const { DOMParser } = require('xmldom');

// Polyfill global DOMParser for the URDF parser
global.DOMParser = DOMParser;

module.exports = {
    run: async (ctx) => {
        const { assert, assertClose, assertVecClose } = ctx;
        let passed = 0;
        let failed = 0;

        const check = (cond) => {
            if (cond) passed++; else failed++;
        };

        console.log('  --- Collision URDF Parsing Tests ---');

        const { parseURDF } = await import('../src/urdf-parser.mjs');

        const dummyURDF = `
        <robot name="test_robot">
            <link name="base_link">
                <collision>
                    <origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>
                    <geometry>
                        <box size="2 4 6"/>
                    </geometry>
                </collision>
            </link>
            <link name="link1">
                <collision>
                    <geometry>
                        <sphere radius="1.5"/>
                    </geometry>
                </collision>
                <collision>
                    <geometry>
                        <cylinder radius="0.5" length="2.0"/>
                    </geometry>
                </collision>
                <collision>
                    <geometry>
                        <mesh filename="dummy.obj" scale="2 4 6"/>
                    </geometry>
                </collision>
            </link>
        </robot>
        `;

        try {
            const urdfData = parseURDF(dummyURDF);
            
            check(assert(urdfData.robotName === 'test_robot', 'Robot name parsed'));
            check(assert(urdfData.links['base_link'].collisions.length === 1, 'base_link has 1 collision'));
            
            const col1 = urdfData.links['base_link'].collisions[0];
            check(assertVecClose(col1.origin.xyz, [1, 2, 3], 1e-6, 'Box origin xyz'));
            check(assertVecClose(col1.origin.rpy, [0.1, 0.2, 0.3], 1e-6, 'Box origin rpy'));
            check(assertVecClose(col1.geometry.min, [-1, -2, -3], 1e-6, 'Box min'));
            check(assertVecClose(col1.geometry.max, [1, 2, 3], 1e-6, 'Box max'));

            check(assert(urdfData.links['link1'].collisions.length === 3, 'link1 has 3 collisions'));
            const colSphere = urdfData.links['link1'].collisions[0];
            check(assertVecClose(colSphere.geometry.min, [-1.5, -1.5, -1.5], 1e-6, 'Sphere min'));
            check(assertVecClose(colSphere.geometry.max, [1.5, 1.5, 1.5], 1e-6, 'Sphere max'));

            const colCyl = urdfData.links['link1'].collisions[1];
            check(assertVecClose(colCyl.geometry.min, [-0.5, -0.5, -1.0], 1e-6, 'Cylinder min'));
            check(assertVecClose(colCyl.geometry.max, [0.5, 0.5, 1.0], 1e-6, 'Cylinder max'));

            const colMesh = urdfData.links['link1'].collisions[2];
            check(assertVecClose(colMesh.geometry.min, [-1.0, -2.0, -3.0], 1e-6, 'Mesh min'));
            check(assertVecClose(colMesh.geometry.max, [1.0, 2.0, 3.0], 1e-6, 'Mesh max'));

            console.log('  --- CollisionChecker Logic Tests ---');
            
            const { buildPinocchioModel } = await import('../src/urdf-parser.mjs');
            const { CollisionChecker } = await import('../src/collision-checker.mjs');
            const { pin } = ctx;

            const model = buildPinocchioModel(pin, urdfData);
            const data = new pin.Data(model);

            // Our dummy robot has a fixed or floating joint depending on parser, but since there's no joint, it just has root_link.
            // Let's create a URDF with an actual joint to test updateCollisions.
            const urdfJoint = `
            <robot name="joint_test">
                <link name="base_link">
                    <collision>
                        <geometry><box size="1 1 1"/></geometry>
                    </collision>
                </link>
                <link name="link1">
                    <collision>
                        <origin xyz="1 0 0" rpy="0 0 0"/>
                        <geometry><box size="1 1 1"/></geometry>
                    </collision>
                </link>
                <link name="link2">
                    <collision>
                        <origin xyz="2 0 0" rpy="0 0 0"/>
                        <geometry><box size="1 1 1"/></geometry>
                    </collision>
                </link>
                <joint name="j1" type="prismatic">
                    <parent link="base_link"/>
                    <child link="link1"/>
                    <origin xyz="1 0 0"/>
                    <axis xyz="1 0 0"/>
                </joint>
                <joint name="j2" type="prismatic">
                    <parent link="link1"/>
                    <child link="link2"/>
                    <origin xyz="1 0 0"/>
                    <axis xyz="1 0 0"/>
                </joint>
            </robot>
            `;
            const urdfData2 = parseURDF(urdfJoint);
            const model2 = buildPinocchioModel(pin, urdfData2);
            const data2 = new pin.Data(model2);

            const checker = new CollisionChecker(pin, model2, data2, urdfData2);
            
            // By default, adjacent parent-child pairs should be ignored. 
            // base_link <-> link1 is adjacent
            // link1 <-> link2 is adjacent
            // base_link <-> link2 is NOT adjacent
            
            // At q=[0,0]:
            // base_link at 0, size 1 -> [-0.5, 0.5]
            // link1 joint at 1.0, collision origin at 1.0 -> absolute collision at 2.0. Box [-0.5, 0.5] -> [1.5, 2.5]
            // link2 joint at (1.0 + 1.0) = 2.0, collision origin at 2.0 -> absolute collision at 4.0. Box -> [3.5, 4.5]
            // So at q=[0,0], base_link [-0.5, 0.5] and link2 [3.5, 4.5] DO NOT intersect.
            let q = new Float64Array(model2.nq).fill(0);
            checker.updateCollisions(q);
            let result = checker.checkCollisions();
            check(assert(result.hasCollision === false, 'No collision at neutral config'));

            // Now move j1 and j2 such that they intersect.
            // Let's set q=[0, -3].
            // link1 joint is at 1.0. absolute collision at 2.0. Box [1.5, 2.5]
            // link2 joint is at 1.0 (j1) + 1.0 (offset) + -3 (j2) = -1.0. 
            // link2 collision origin is 2.0 relative to link2 frame. So absolute is -1.0 + 2.0 = 1.0. Box [0.5, 1.5]
            // wait, base_link is [-0.5, 0.5]. They don't intersect base link yet. Let's make j2 = -4.
            // link2 joint = 1.0 + 1.0 - 4 = -2.0. abs collision = 0.0. Box [-0.5, 0.5].
            // Intersects base_link perfectly!
            q[0] = 0;
            q[1] = -4;
            checker.updateCollisions(q);
            result = checker.checkCollisions();
            check(assert(result.hasCollision === true, 'Collision detected when folded back'));
            if (result.hasCollision) {
                check(assert(result.contacts.length > 0, 'Contacts array is populated'));
                const contact = result.contacts[0];
                check(assert((contact.linkA === 'base_link' && contact.linkB === 'link2') || 
                             (contact.linkA === 'link2' && contact.linkB === 'base_link'), 
                             'Correct collision pair detected'));
            }

            // Test setting ignored pairs explicitly
            checker.setIgnoredPairs([['base_link', 'link2']]);
            checker.updateCollisions(q);
            result = checker.checkCollisions();
            check(assert(result.hasCollision === false, 'Collision ignored after setting ignored pairs'));

            console.log('  --- IK Loop Performance Test ---');
            const numIterations = 1000;
            const t0 = performance.now();
            for (let i = 0; i < numIterations; i++) {
                q[0] = Math.random();
                q[1] = Math.random();
                checker.updateCollisions(q);
                checker.checkCollisions();
            }
            const t1 = performance.now();
            const elapsed = t1 - t0;
            const timePerConfig = elapsed / numIterations;
            console.log(`      Average time per config: ${timePerConfig.toFixed(4)} ms`);
            check(assert(timePerConfig < 1.0, `Execution time per config < 1ms (${timePerConfig.toFixed(4)}ms)`));

        } catch (e) {
            console.log(`      ❌ Error: ${e.message}`);
            failed++;
        }

        return { passed, failed };
    }
};
