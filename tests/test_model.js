/**
 * Pinocchio WASM — Model Tests
 */

module.exports = {
    run: async (ctx) => {
        const { pin, assert } = ctx;
        let passed = 0;
        let failed = 0;

        function check(cond) {
            if (cond) passed++; else failed++;
        }

        console.log('  --- Joint Creation Tests ---');

        const model = new pin.Model();
        const id = pin.SE3.identity();

        // 1. Fixed
        try {
            const idx = pin.addJoint(model, 0, pin.JointModelFixed(), id, "fixed_joint");
            check(assert(idx > 0, `Added Fixed joint (id=${idx})`));
        } catch (e) { console.error(e); failed++; }

        // 2. Revolute (RX, RY, RZ)
        try {
            const rx = pin.addJoint(model, 0, pin.JointModelRX(), id, "rx");
            check(assert(rx > 0, `Added RX (id=${rx})`));

            const ry = pin.addJoint(model, 0, pin.JointModelRY(), id, "ry");
            check(assert(ry > 0, `Added RY (id=${ry})`));

            const rz = pin.addJoint(model, 0, pin.JointModelRZ(), id, "rz");
            check(assert(rz > 0, `Added RZ (id=${rz})`));
        } catch (e) { console.error(e); failed++; }

        // 3. Prismatic (PX, PY, PZ)
        try {
            const px = pin.addJoint(model, 0, pin.JointModelPX(), id, "px");
            check(assert(px > 0, `Added PX (id=${px})`));
        } catch (e) { console.error(e); failed++; }

        // 4. FreeFlyer
        try {
            const ff = pin.addJoint(model, 0, pin.JointModelFreeFlyer(), id, "ff");
            check(assert(ff > 0, `Added FreeFlyer (id=${ff})`));
        } catch (e) { console.error(e); failed++; }

        // 5. Unaligned
        try {
            const ru = pin.addJoint(model, 0, pin.JointModelRevoluteUnaligned(1, 1, 0), id, "ru");
            check(assert(ru > 0, `Added RevoluteUnaligned (id=${ru})`));
        } catch (e) { console.error(e); failed++; }

        console.log('  --- Model Properties ---');
        // We added: fixed(0dof), rx(1), ry(1), rz(1), px(1), ff(7), ru(1)
        // nq: 0 + 1 + 1 + 1 + 1 + 7 + 1 = 12?
        // nv: 0 + 1 + 1 + 1 + 1 + 6 + 1 = 11?
        // njoints: 1 (universe) + 7 added = 8?
        // Wait, FreeFlyer is 7 nq, 6 nv.
        // Revolute is 1, 1.
        // Fixed is 0, 0.

        // Let's verify
        const nq_exp = 1 + 1 + 1 + 1 + 7 + 1; // 12
        const nv_exp = 1 + 1 + 1 + 1 + 6 + 1; // 11
        const nj_exp = 1 + 7;                 // 8

        check(assert(model.nq === nq_exp, `nq matches (${model.nq} vs ${nq_exp})`));
        check(assert(model.nv === nv_exp, `nv matches (${model.nv} vs ${nv_exp})`));
        check(assert(model.njoints === nj_exp, `njoints matches (${model.njoints} vs ${nj_exp})`));

        return { passed, failed };
    }
};
