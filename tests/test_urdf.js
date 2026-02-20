const fs = require('fs');
const path = require('path');
const { DOMParser } = require('xmldom');

// Polyfill global DOMParser for the URDF parser
global.DOMParser = DOMParser;

module.exports = {
    run: async (ctx) => {
        const { pin, assert } = ctx;
        let passed = 0;
        let failed = 0;

        // Helper to integrate with our passed/failed counters while using runner's logging
        const check = (cond) => {
            if (cond) passed++; else failed++;
        };

        console.log('  --- URDF Loading Tests ---');

        // Load the ESM parser dynamically
        const { parseURDF, buildPinocchioModel } = await import('../src/urdf-parser.mjs');

        const urdfDir = path.join(__dirname, '../urdf');
        const testFiles = [
            {
                path: 'abb_irb120_support/urdf/abbIrb120.urdf',
                expectedJoints: 6 + 1 // + universe
            },
            {
                path: 'kuka_kr210_support/urdf/kr210l150.urdf',
                expectedJoints: 6 + 1
            },
            {
                path: 'yumi_description/urdf/abbYuMi.urdf',
                expectedJoints: 14 + 1 // YuMi is a dual arm, usually 7+7.
            }
        ];

        for (const file of testFiles) {
            const fullPath = path.join(urdfDir, file.path);

            if (!fs.existsSync(fullPath)) {
                console.log(`      ⚠️ File not found: ${fullPath}`);
                // Don't fail the suite if file is missing, just warn? 
                // Or fail? User specifically asked for these tests.
                // Let's fail.
                check(assert(false, `File exists: ${file.path}`));
                continue;
            }

            try {
                console.log(`    🔹 Loading ${file.path}...`);
                const xml = fs.readFileSync(fullPath, 'utf8');
                const urdfData = parseURDF(xml);
                const model = buildPinocchioModel(pin, urdfData);
                const data = new pin.Data(model);

                check(assert(model.nq > 0, `Model built: ${urdfData.robotName || 'unnamed'}`));
                console.log(`      → njoints: ${model.njoints}, nq: ${model.nq}, nv: ${model.nv}`);

                // Basic checks
                if (file.expectedJoints) {
                    // Approximate check because fixed joints might vary in URDF
                    check(assert(model.njoints >= file.expectedJoints, `Joint count >= ${file.expectedJoints}`));
                }

                // Run algorithms
                const q = new Float64Array(model.nq).fill(0);
                const v = new Float64Array(model.nv).fill(0);
                const a = new Float64Array(model.nv).fill(0);

                // Neutral config might be better for some robots
                // but zero is usually safe for basic RNEA math check
                pin.rnea(model, data, q, v, a);
                check(assert(true, 'RNEA runs without error'));

                const com = pin.centerOfMass(model, data, q);
                check(assert(com.length === 3, 'COM computed'));

            } catch (e) {
                console.log(`      ❌ Error: ${e.message}`);
                failed++;
            }
        }

        return { passed, failed };
    }
};
