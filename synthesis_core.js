// Three-position double rocker synthesis using numerical optimization
function doubleRockerSynthesis(groundLen, inputAngles, outputAngles, tol=1e-10, maxIter=5000) {
    // Normalize ground link to 1 for optimization
    const r1_norm = 1.0;
    // Cost function: difference in coupler lengths at three positions
    function cost(x) {
        const r2 = x[0];
        const r4 = x[1];
        const A = [-r1_norm/2, 0];
        const B = [r1_norm/2, 0];
        const theta = inputAngles.map(a => a * Math.PI / 180);
        const phi = outputAngles.map(a => a * Math.PI / 180);
        function Ai(i) {
            return [A[0] + r2 * Math.cos(theta[i]), A[1] + r2 * Math.sin(theta[i])];
        }
        function Bi(i) {
            return [B[0] + r4 * Math.cos(phi[i]), B[1] + r4 * Math.sin(phi[i])];
        }
        const d1 = Math.hypot(Ai(0)[0] - Bi(0)[0], Ai(0)[1] - Bi(0)[1]);
        const d2 = Math.hypot(Ai(1)[0] - Bi(1)[0], Ai(1)[1] - Bi(1)[1]);
        const d3 = Math.hypot(Ai(2)[0] - Bi(2)[0], Ai(2)[1] - Bi(2)[1]);
        return (d1 - d2)**2 + (d2 - d3)**2;
    }
    // Simple Nelder-Mead optimizer
    let x = [1, 1];
    let step = 0.05;
    for (let iter = 0; iter < maxIter; iter++) {
        let best = cost(x);
        let improved = false;
        for (let i = 0; i < 2; i++) {
            for (let dir of [-1, 1]) {
                let xnew = x.slice();
                xnew[i] += dir * step;
                if (xnew[i] <= 0) continue;
                let c = cost(xnew);
                if (c < best) {
                    x = xnew;
                    best = c;
                    improved = true;
                }
            }
        }
        if (!improved) step *= 0.5;
        if (step < tol) break;
    }
    // Compute final coupler length (normalized)
    const r2_norm = x[0];
    const r4_norm = x[1];
    const A = [-r1_norm/2, 0];
    const B = [r1_norm/2, 0];
    const theta = inputAngles.map(a => a * Math.PI / 180);
    const phi = outputAngles.map(a => a * Math.PI / 180);
    function Ai(i) {
        return [A[0] + r2_norm * Math.cos(theta[i]), A[1] + r2_norm * Math.sin(theta[i])];
    }
    function Bi(i) {
        return [B[0] + r4_norm * Math.cos(phi[i]), B[1] + r4_norm * Math.sin(phi[i])];
    }
    const d1 = Math.hypot(Ai(0)[0] - Bi(0)[0], Ai(0)[1] - Bi(0)[1]);
    const d2 = Math.hypot(Ai(1)[0] - Bi(1)[0], Ai(1)[1] - Bi(1)[1]);
    const d3 = Math.hypot(Ai(2)[0] - Bi(2)[0], Ai(2)[1] - Bi(2)[1]);
    const r3_norm = (d1 + d2 + d3) / 3;
    // Scale all lengths to desired ground link
    const scale = groundLen / r1_norm;
    return {
        inputLen: r2_norm * scale,
        outputLen: r4_norm * scale,
        couplerLen: r3_norm * scale,
        d1: d1 * scale,
        d2: d2 * scale,
        d3: d3 * scale
    };
}

// synthesis_core.js
// Core synthesis and kinematic functions for 4-bar linkage (no browser dependencies)

// Newton-Raphson solver for 4-bar synthesis
function newtonSolver(groundLen, inputAngles, outputAngles, tol=1e-10, maxIter=1000) {
    // Initial guess: use lengths from initial drawing (first design position)
    const theta1 = inputAngles.map(a => a * Math.PI / 180);
    const theta2 = outputAngles.map(a => a * Math.PI / 180);
    const A = [-groundLen/2, 0];
    const B = [groundLen/2, 0];
    // Calculate initial input and output link lengths from first position
    const inputLen_init = Math.hypot(
        Math.cos(theta1[0]),
        Math.sin(theta1[0])
    );
    const outputLen_init = Math.hypot(
        Math.cos(theta2[0]),
        Math.sin(theta2[0])
    );
    let inputLen = inputLen_init * groundLen;
    let outputLen = outputLen_init * groundLen;
    function couplerLenAt(i, inputLen, outputLen) {
        const Cx = A[0] + inputLen * Math.cos(theta1[i]);
        const Cy = A[1] + inputLen * Math.sin(theta1[i]);
        const Dx = B[0] + outputLen * Math.cos(theta2[i]);
        const Dy = B[1] + outputLen * Math.sin(theta2[i]);
        return Math.sqrt((Cx - Dx)**2 + (Cy - Dy)**2);
    }
    let iter = 0;
    while (iter < maxIter) {
        // Compute coupler lengths at each position
        const L0 = couplerLenAt(0, inputLen, outputLen);
        const L1 = couplerLenAt(1, inputLen, outputLen);
        const L2 = couplerLenAt(2, inputLen, outputLen);
        // Residuals: want L0 = L1 = L2
        const F1 = L1 - L0;
        const F2 = L2 - L1;
        // Jacobian numerically
        const h = 1e-6;
        const dF1_dInput = (couplerLenAt(1, inputLen+h, outputLen) - couplerLenAt(0, inputLen+h, outputLen) - F1) / h;
        const dF1_dOutput = (couplerLenAt(1, inputLen, outputLen+h) - couplerLenAt(0, inputLen, outputLen+h) - F1) / h;
        const dF2_dInput = (couplerLenAt(2, inputLen+h, outputLen) - couplerLenAt(1, inputLen+h, outputLen) - F2) / h;
        const dF2_dOutput = (couplerLenAt(2, inputLen, outputLen+h) - couplerLenAt(1, inputLen, outputLen+h) - F2) / h;
        // 2x2 Jacobian
        const J = [
            [dF1_dInput, dF1_dOutput],
            [dF2_dInput, dF2_dOutput]
        ];
        // Solve J * dx = -F
        const det = J[0][0]*J[1][1] - J[0][1]*J[1][0];
        if (Math.abs(det) < 1e-12) break;
        const invJ = [
            [ J[1][1]/det, -J[0][1]/det],
            [-J[1][0]/det,  J[0][0]/det]
        ];
        const dx = [
            -(invJ[0][0]*F1 + invJ[0][1]*F2),
            -(invJ[1][0]*F1 + invJ[1][1]*F2)
        ];
        inputLen += dx[0];
        outputLen += dx[1];
        if (Math.abs(F1) < tol && Math.abs(F2) < tol) break;
        iter++;
    }
    // Final coupler length
    const couplerLen = couplerLenAt(0, inputLen, outputLen);
    return {inputLen, outputLen, couplerLen, iterations: iter};
}

const leastSquaresSolver = function(groundLen, inputAngles, outputAngles) {
    if (inputAngles.length === 3 && outputAngles.length === 3) {
        // Use Newton solver for three positions
        return newtonSolver(groundLen, inputAngles, outputAngles);
    } else {
        // Fallback to grid search for other cases
        let best = null;
        let minErr = Infinity;
        for (let inputLen = 40; inputLen <= 200; inputLen += 5) {
            for (let outputLen = 40; outputLen <= 200; outputLen += 5) {
                for (let couplerLen = 40; couplerLen <= 200; couplerLen += 5) {
                    let err = 0;
                    for (let i = 0; i < inputAngles.length; i++) {
                        const theta1 = inputAngles[i] * Math.PI / 180;
                        const theta2 = outputAngles[i] * Math.PI / 180;
                        const val = inputLen**2 + outputLen**2 - 2*inputLen*outputLen*Math.cos(theta1 - theta2) - couplerLen**2;
                        err += val*val;
                    }
                    if (err < minErr) {
                        minErr = err;
                        best = {inputLen, outputLen, couplerLen};
                    }
                }
            }
        }
        return best;
    }
// ...function ends here, no extra closing brace...
}

function interpolateOutputAngle(inputAngle, solution, groundLength) {
    const r1 = groundLength;
    const r2 = solution.inputLen;
    const r3 = solution.couplerLen;
    const r4 = solution.outputLen;
    const theta2 = inputAngle * Math.PI / 180;
    const Ax = -r1/2, Ay = 0;
    const Bx = r1/2, By = 0;
    const Cx = Ax + r2 * Math.cos(theta2);
    const Cy = Ay + r2 * Math.sin(theta2);
    const dx = Cx - Bx;
    const dy = Cy - By;
    const d = Math.sqrt(dx*dx + dy*dy);
    if (d > r3 + r4 || d < Math.abs(r3 - r4)) {
        return 0;
    }
    const a = (r4*r4 - r3*r3 + d*d) / (2*d);
    const h = Math.sqrt(Math.max(0, r4*r4 - a*a));
    const xm = Bx + a * (dx) / d;
    const ym = By + a * (dy) / d;
    const xs1 = xm + h * (dy) / d;
    const ys1 = ym - h * (dx) / d;
    const Dx = xs1;
    const Dy = ys1;
    const theta4 = Math.atan2(Dy - By, Dx - Bx);
    return theta4 * 180 / Math.PI;
}

module.exports = { leastSquaresSolver, interpolateOutputAngle, doubleRockerSynthesis };
