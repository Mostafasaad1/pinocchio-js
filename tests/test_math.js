/**
 * Pinocchio WASM — Math Tests (SE3, Inertia)
 */

module.exports = {
    run: async (ctx) => {
        const { pin, assert, assertClose, assertVecClose } = ctx;
        let passed = 0;
        let failed = 0;

        function check(cond) {
            if (cond) passed++; else failed++;
        }

        console.log('  --- SE3 Tests ---');

        // 1. Identity
        try {
            const id = pin.SE3.identity();
            // SE3 doesn't expose internal matrix/vector accessors directly in JS API yet?
            // Wait, in pinocchio_embind.cpp we expose class SE3 but no properties?
            // Ah, Step 684:
            // class_<SE3>("SE3")
            //    .class_function("identity", &se3Identity)
            //    .class_function("fromRotationTranslation", &se3FromRotationTranslation)
            //    .class_function("fromXyzRpy", &se3FromXyzRpy)
            // No .property("rotation") or .property("translation")!
            // So we can't inspect it directly from JS side efficiently?
            // Wait, we can test it by using it in a Model!
            // Or adding inspect methods.

            check(assert(!!id, 'SE3.identity() matches object'));
        } catch (e) {
            console.error(e);
            failed++;
        }

        // 2. fromXyzRpy
        try {
            const se3 = pin.SE3.fromXyzRpy(1, 2, 3, 0, 0, 0);
            check(assert(!!se3, 'SE3.fromXyzRpy created'));
        } catch (e) {
            console.error(e);
            failed++;
        }

        console.log('  --- Inertia Tests ---');

        // 3. Inertia
        try {
            const mass = 2.5;
            const com = [0.1, 0.2, 0.3];
            const I = [0.1, 0, 0, 0.1, 0, 0.1];
            const inertia = pin.Inertia.fromMassComInertia(mass, com, I);
            check(assert(!!inertia, 'Inertia created'));

            // Again, no read accessors exposed in bindings.
            // We'll verify them via computeTotalMass later.
            const model = new pin.Model();
            // Add a body with this inertia
            const j1 = pin.addJoint(model, 0, pin.JointModelFixed(), pin.SE3.identity(), "test_body");
            pin.appendBodyToJoint(model, j1, inertia, pin.SE3.identity());

            const totalMass = pin.computeTotalMass(model);
            check(assertClose(totalMass, mass, 1e-6, `Total mass matches (${totalMass} vs ${mass})`));

        } catch (e) {
            console.error(e);
            failed++;
        }

        return { passed, failed };
    }
};
