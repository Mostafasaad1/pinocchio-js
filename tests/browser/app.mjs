import { browserTests } from './test-suite.mjs';
import { parseURDF, buildPinocchioModel } from '../../src/urdf-parser.mjs';

const statusEl = document.getElementById('global-status');
const testListEl = document.getElementById('test-list');
const runAllBtn = document.getElementById('run-all-btn');

let pinModule = null;

function renderTestCard(test) {
    const el = document.createElement('div');
    el.className = 'test-card';
    el.id = `card-${test.id}`;

    el.innerHTML = `
        <div class="test-header">
            <h3 class="test-title">${test.title}</h3>
            <span class="badge badge-pending" id="badge-${test.id}">Pending</span>
        </div>
        <pre class="test-output" id="output-${test.id}">Waiting to run...</pre>
        <div class="test-timing" id="timing-${test.id}" style="display:none;"></div>
    `;

    return el;
}

async function runTest(test, pin) {
    const badge = document.getElementById(`badge-${test.id}`);
    const output = document.getElementById(`output-${test.id}`);
    const timing = document.getElementById(`timing-${test.id}`);

    badge.className = 'badge badge-running';
    badge.textContent = 'Running';

    try {
        const res = await test.run(pin);
        badge.className = 'badge badge-pass';
        badge.textContent = 'Pass';
        output.textContent = res.result || 'Completed successfully.';

        if (res.timing) {
            timing.textContent = res.timing;
            timing.style.display = 'block';
        } else {
            timing.style.display = 'none';
        }
    } catch (err) {
        badge.className = 'badge badge-fail';
        badge.textContent = 'Fail';
        output.textContent = `Error: ${err.message}\n${err.stack || ''}`;
        timing.style.display = 'none';
        console.error(`Test ${test.id} failed:`, err);
    }
}

async function runAllTests() {
    if (!pinModule) return;
    if (runAllBtn) runAllBtn.disabled = true;

    for (const test of browserTests) {
        await runTest(test, pinModule);
    }

    if (runAllBtn) runAllBtn.disabled = false;
}

function initUrdfLoader(pin) {
    const urdfBtn = document.getElementById('urdf-btn');
    const urdfInput = document.getElementById('urdf-input');
    const urdfOutput = document.getElementById('urdf-output');
    const urdfBadge = document.getElementById('urdf-badge');

    if (!urdfBtn || !urdfInput || !urdfOutput) return;

    urdfBtn.addEventListener('click', () => {
        const urdfStr = urdfInput.value.trim();
        if (!urdfStr) {
            urdfOutput.textContent = 'Please enter URDF XML above.';
            return;
        }

        try {
            const urdfData = parseURDF(urdfStr);
            const model = buildPinocchioModel(pin, urdfData);
            const data = new pin.Data(model);

            const q = pin.neutralConfiguration(model);
            const v = new Float64Array(model.nv).fill(0);
            const a = new Float64Array(model.nv).fill(0);

            const tau = pin.rnea(model, data, q, v, a);
            const com = pin.centerOfMass(model, data, q);

            if (urdfBadge) {
                urdfBadge.className = 'badge badge-pass';
                urdfBadge.textContent = 'Success';
            }

            urdfOutput.textContent =
                `Loaded robot: "${urdfData.robotName}"\n` +
                `Joints: ${model.njoints} | nq: ${model.nq} | nv: ${model.nv}\n` +
                `Gravity torques: [${Array.from(tau).slice(0, 6).map(x => x.toFixed(4)).join(', ')}${model.nv > 6 ? ', ...' : ''}]\n` +
                `Center of mass:  [${Array.from(com).map(x => x.toFixed(4)).join(', ')}]`;
        } catch (e) {
            if (urdfBadge) {
                urdfBadge.className = 'badge badge-fail';
                urdfBadge.textContent = 'Error';
            }
            urdfOutput.textContent = `Error: ${e.message}`;
        }
    });
}

async function init() {
    // Render initial list
    if (testListEl) {
        testListEl.innerHTML = '';
        for (const test of browserTests) {
            testListEl.appendChild(renderTestCard(test));
        }
    }

    try {
        if (typeof PinocchioModule !== 'function') {
            throw new Error('PinocchioModule not found. Ensure pinocchio.js is loaded.');
        }

        statusEl.innerHTML = '<span class="status-indicator status-loading"></span> Initializing WASM module...';
        pinModule = await PinocchioModule();
        statusEl.innerHTML = '<span class="status-indicator status-ready"></span> WASM Ready';

        initUrdfLoader(pinModule);

        if (runAllBtn) {
            runAllBtn.disabled = false;
            runAllBtn.addEventListener('click', runAllTests);
        }

        // Auto-run tests
        await runAllTests();
    } catch (err) {
        statusEl.innerHTML = `<span class="status-indicator status-error"></span> Failed to load WASM: ${err.message}`;
        console.error(err);
    }
}

init();
