const createPin = require('../build/pinocchio.js');

function assert(cond, msg) {
    if (!cond) { console.error('  FAIL:', msg); return false; }
    console.log('  OK:', msg); return true;
}
function assertClose(a, b, tol, msg) {
    const diff = Math.abs(a - b);
    const ok = diff <= tol;
    console.log(`  ${ok ? 'OK' : 'FAIL'}: ${msg} (diff=${diff.toExponential(2)})`);
    return ok;
}
function assertVecClose(a, b, tol, msg) {
    let maxDiff = 0;
    for (let i = 0; i < a.length; i++) maxDiff = Math.max(maxDiff, Math.abs(a[i] - b[i]));
    const ok = maxDiff <= tol;
    console.log(`  ${ok ? 'OK' : 'FAIL'}: ${msg} (max diff=${maxDiff.toExponential(2)})`);
    return ok;
}

createPin().then(async pin => {
    console.log('');
    console.log('=== ABA External Forces — Deep Verification Suite ===');
    console.log('');
    const test = require('./test_aba_fext_deep.js');
    const { passed, failed } = await test.run({ pin, assert, assertClose, assertVecClose });
    console.log('');
    console.log(`Deep Test Summary: ${passed} passed, ${failed} failed`);
    process.exit(failed > 0 ? 1 : 0);
}).catch(e => { console.error('Fatal:', e); process.exit(1); });
