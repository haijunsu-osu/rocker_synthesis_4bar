// Node.js test for 4-bar synthesis validation
const { leastSquaresSolver, interpolateOutputAngle, doubleRockerSynthesis } = require('./synthesis_core');

// Example test case
const groundLength = 200;
const inputAngles = [30, 60, 90];
const outputAngles = [45, 75, 105];


function validateSolution(solution) {
    if (!solution) return;
    const r1 = groundLength;
    const r2 = solution.inputLen;
    const r3 = solution.couplerLen;
    const r4 = solution.outputLen;
    const Ax = -r1/2, Ay = 0;
    const Bx = r1/2, By = 0;
    let allMatch = true;
    for (let i = 0; i < inputAngles.length; i++) {
        const theta2 = inputAngles[i];
        const theta4 = interpolateOutputAngle(theta2, solution, groundLength);
        const theta2_rad = theta2 * Math.PI / 180;
        const theta4_rad = theta4 * Math.PI / 180;
        const Cx = Ax + r2 * Math.cos(theta2_rad);
        const Cy = Ay + r2 * Math.sin(theta2_rad);
        const Dx = Bx + r4 * Math.cos(theta4_rad);
        const Dy = By + r4 * Math.sin(theta4_rad);
        const dist = Math.sqrt((Cx - Dx)**2 + (Cy - Dy)**2);
        const outAngle = theta4;
        const expectedOutAngle = outputAngles[i];
        const angleError = Math.abs(outAngle - expectedOutAngle);
        const lengthError = Math.abs(dist - r3);
        console.log(`Task ${i+1}: Input=${theta2.toFixed(2)} Output=${outAngle.toFixed(2)} (expected ${expectedOutAngle.toFixed(2)}) Coupler=${dist.toFixed(2)} (expected ${r3.toFixed(2)}) AngleErr=${angleError.toFixed(2)} LenErr=${lengthError.toFixed(2)}`);
        if (angleError > 1.0 || lengthError > 1.0) allMatch = false;
    }
    if (allMatch) {
        console.log('Validation PASSED: Linkage reaches all specified task positions.');
    } else {
        console.log('Validation FAILED: Linkage does not reach all specified task positions.');
    }
}


console.log('Testing doubleRockerSynthesis solver:');
const drSolution = doubleRockerSynthesis(groundLength, inputAngles, outputAngles);
console.log(`Input link: ${drSolution.inputLen.toFixed(4)}`);
console.log(`Output link: ${drSolution.outputLen.toFixed(4)}`);
console.log(`Coupler link: ${drSolution.couplerLen.toFixed(4)}`);
console.log(`Coupler lengths at positions: ${drSolution.d1.toFixed(8)}, ${drSolution.d2.toFixed(8)}, ${drSolution.d3.toFixed(8)}`);
console.log(`Coupler length errors: ${(drSolution.d1-drSolution.d2).toExponential(2)}, ${(drSolution.d2-drSolution.d3).toExponential(2)}`);
if (Math.abs(drSolution.d1-drSolution.d2)<1e-6 && Math.abs(drSolution.d2-drSolution.d3)<1e-6) {
    console.log('Validation PASSED: Coupler length is constant at all positions.');
} else {
    console.log('Validation FAILED: Coupler length is not constant.');
}
