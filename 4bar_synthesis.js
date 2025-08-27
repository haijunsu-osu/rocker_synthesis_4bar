
// 4bar_synthesis.js
// Interactive synthesis of planar 4-bar linkages
// Use optimization-based solver from synthesis_core.js
const { doubleRockerSynthesis } = require('./synthesis_core');
const { leastSquaresSolver } = require('./synthesis_core');

const canvas = document.getElementById('linkage-canvas');
const ctx = canvas.getContext('2d');
const synthesizeBtn = document.getElementById('synthesizeBtn');
const playPauseBtn = document.getElementById('playPauseBtn');

let animationId = null;
let isPlaying = false;
let solution = null;
let inputAngles = [30, 60, 90];
let outputAngles = [45, 75, 105];
let groundLength = 200;
let currentInputAngle = 30;
let animationStep = 0;
let slider = null;
const linkLengthsDiv = document.getElementById('linkLengths');
let plotCanvas = null;
let plotCtx = null;
const currentAnglesDiv = document.getElementById('currentAngles');

function parseAngles(str) {
    return str.split(',').map(s => parseFloat(s.trim())).filter(a => !isNaN(a));
}

function drawGrid() {
    ctx.save();
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 1;
    const spacing = 50;
    for (let x = 0; x < canvas.width; x += spacing) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
    }
    for (let y = 0; y < canvas.height; y += spacing) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
    }
    ctx.restore();
}

function toCanvasCoords(x, y) {
    // (0,0) at center, y up
    return [canvas.width / 2 + x, canvas.height / 2 - y];
}

function drawLinkage(groundLen, inputLen, outputLen, couplerLen, inputAngle, outputAngle, highlight=false) {
    // Ground link: from (-groundLen/2, 0) to (groundLen/2, 0)
    const A = [-groundLen/2, 0];
    const B = [groundLen/2, 0];
    // Input link: from A at inputAngle
    const theta1 = inputAngle * Math.PI / 180;
    const C = [A[0] + inputLen * Math.cos(theta1), A[1] + inputLen * Math.sin(theta1)];
    // Output link: from B at outputAngle
    const theta2 = outputAngle * Math.PI / 180;
    const D = [B[0] + outputLen * Math.cos(theta2), B[1] + outputLen * Math.sin(theta2)];
    // Coupler: C to D
    ctx.save();
    // Draw ground
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(...toCanvasCoords(...A));
    ctx.lineTo(...toCanvasCoords(...B));
    ctx.stroke();
    // Draw input
    ctx.strokeStyle = highlight ? '#1976d2' : '#888';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(...toCanvasCoords(...A));
    ctx.lineTo(...toCanvasCoords(...C));
    ctx.stroke();
    // Draw output
    ctx.strokeStyle = highlight ? '#d32f2f' : '#888';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(...toCanvasCoords(...B));
    ctx.lineTo(...toCanvasCoords(...D));
    ctx.stroke();
    // Draw coupler
    ctx.strokeStyle = '#388e3c';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(...toCanvasCoords(...C));
    ctx.lineTo(...toCanvasCoords(...D));
    ctx.stroke();
    // Draw joints
    [A, B, C, D].forEach(([x, y], i) => {
        ctx.beginPath();
        ctx.arc(...toCanvasCoords(x, y), 7, 0, 2 * Math.PI);
        ctx.fillStyle = i < 2 ? '#333' : '#fff';
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 2;
        ctx.fill();
        ctx.stroke();
    });
    ctx.restore();
}

function drawSpecifiedPositions() {
    drawGrid();
    // Draw input/output links for each specified position using computed solution if available
    if (solution) {
        for (let i = 0; i < inputAngles.length; i++) {
            drawLinkage(groundLength, solution.inputLen, solution.outputLen, solution.couplerLen, inputAngles[i], outputAngles[i], true);
        }
    } else {
        // Fallback to default visualization if no solution yet
        for (let i = 0; i < inputAngles.length; i++) {
            drawLinkage(groundLength, 60, 60, 80, inputAngles[i], outputAngles[i], true);
        }
    }
}

// Use doubleRockerSynthesis for three positions, otherwise fallback to leastSquaresSolver from synthesis_core
// Removed duplicate leastSquaresSolver declaration to fix redeclaration error
// The leastSquaresSolver function is now imported from synthesis_core.js

function synthesize() {
    // Validate the solution by kinematic analysis
    function validateSolution() {
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
            const theta4 = interpolateOutputAngle(theta2);
            // Compute C and D
            const theta2_rad = theta2 * Math.PI / 180;
            const theta4_rad = theta4 * Math.PI / 180;
            const Cx = Ax + r2 * Math.cos(theta2_rad);
            const Cy = Ay + r2 * Math.sin(theta2_rad);
            const Dx = Bx + r4 * Math.cos(theta4_rad);
            const Dy = By + r4 * Math.sin(theta4_rad);
            // Coupler length
            const dist = Math.sqrt((Cx - Dx)**2 + (Cy - Dy)**2);
            // Should match r3
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
    validateSolution();
    groundLength = parseFloat(document.getElementById('groundLength').value);
    inputAngles = parseAngles(document.getElementById('inputAngles').value);
    outputAngles = parseAngles(document.getElementById('outputAngles').value);
    if (inputAngles.length !== outputAngles.length || inputAngles.length < 2) {
        alert('Please enter the same number of input and output angles (at least 2 pairs).');
        return;
    }
    solution = leastSquaresSolver(groundLength, inputAngles, outputAngles);
    if (!solution) {
        alert('No solution found. Try different angles or ground length.');
        return;
    }
    // Show link lengths
    linkLengthsDiv.innerHTML = `<b>Link Lengths:</b> Ground: ${groundLength.toFixed(2)}, Input: ${solution.inputLen.toFixed(2)}, Output: ${solution.outputLen.toFixed(2)}, Coupler: ${solution.couplerLen.toFixed(2)}`;
    // Draw both original task positions and solution linkage overlay
    drawSpecifiedPositions();
    drawLinkage(groundLength, solution.inputLen, solution.outputLen, solution.couplerLen, currentInputAngle, interpolateOutputAngle(currentInputAngle), false);
    animationStep = 0;
    // Update slider range and value
    if (slider) {
        slider.min = Math.min(...inputAngles);
        slider.max = Math.max(...inputAngles);
        slider.value = currentInputAngle;
    }
    // Initialize plot canvas and context after DOM update
    plotCanvas = document.getElementById('plot-canvas');
    plotCtx = plotCanvas ? plotCanvas.getContext('2d') : null;
    drawPlot();
    updateCurrentAngles();
}

function animateLinkage() {
    if (!solution) return;
    // Animate input angle from min to max
    const minAngle = Math.min(...inputAngles);
    const maxAngle = Math.max(...inputAngles);
    currentInputAngle += 1;
    if (currentInputAngle > maxAngle) currentInputAngle = minAngle;
    if (slider) slider.value = currentInputAngle;
    // Draw both original task positions and solution linkage overlay
    drawSpecifiedPositions();
    drawLinkage(groundLength, solution.inputLen, solution.outputLen, solution.couplerLen, currentInputAngle, interpolateOutputAngle(currentInputAngle), false);
    animationId = requestAnimationFrame(animateLinkage);
    updateCurrentAngles();
    drawPlot();
}
function interpolateOutputAngle(inputAngle) {
    // Four-bar kinematic analysis: solve for output angle given input angle and link lengths
    if (!solution) return 0;
    const r1 = groundLength;
    const r2 = solution.inputLen;
    const r3 = solution.couplerLen;
    const r4 = solution.outputLen;
    const theta2 = inputAngle * Math.PI / 180;
    // Vector loop closure: r1 + r2*e^(i*theta2) = r4*e^(i*theta4) + r3*e^(i*theta3)
    // Solve for theta4 (output link angle) using law of cosines
    // Compute position of C (input link end)
    const Ax = -r1/2, Ay = 0;
    const Bx = r1/2, By = 0;
    const Cx = Ax + r2 * Math.cos(theta2);
    const Cy = Ay + r2 * Math.sin(theta2);
    // D must be at distance r4 from B and r3 from C
    // Find intersection of two circles
    const dx = Cx - Bx;
    const dy = Cy - By;
    const d = Math.sqrt(dx*dx + dy*dy);
    if (d > r3 + r4 || d < Math.abs(r3 - r4)) {
        // No solution
        return 0;
    }
    // Circle intersection math
    const a = (r4*r4 - r3*r3 + d*d) / (2*d);
    const h = Math.sqrt(Math.max(0, r4*r4 - a*a));
    const xm = Bx + a * (dx) / d;
    const ym = By + a * (dy) / d;
    // Two possible solutions (open/closed)
    const xs1 = xm + h * (dy) / d;
    const ys1 = ym - h * (dx) / d;
    // Use the solution with positive y (above ground)
    const Dx = xs1;
    const Dy = ys1;
    // Output angle
    const theta4 = Math.atan2(Dy - By, Dx - Bx);
    return theta4 * 180 / Math.PI;
}

synthesizeBtn.onclick = synthesize;
playPauseBtn.onclick = function() {
    if (!solution) return;
    if (isPlaying) {
        cancelAnimationFrame(animationId);
        playPauseBtn.textContent = 'Play';
        isPlaying = false;
    } else {
        isPlaying = true;
        playPauseBtn.textContent = 'Pause';
        animateLinkage();
    }
};

// Add slider for driver link position
function addSlider() {
    const controlsDiv = document.querySelector('.controls');
    slider = document.createElement('input');
    slider.type = 'range';
    slider.min = Math.min(...inputAngles);
    slider.max = Math.max(...inputAngles);
    slider.value = currentInputAngle;
    slider.step = 1;
    slider.style.width = '300px';
    slider.id = 'driverSlider';
    slider.oninput = function() {
        currentInputAngle = parseFloat(slider.value);
        if (solution) {
            drawSpecifiedPositions();
            drawLinkage(groundLength, solution.inputLen, solution.outputLen, solution.couplerLen, currentInputAngle, interpolateOutputAngle(currentInputAngle), false);
            updateCurrentAngles();
            // Ensure plotCanvas and plotCtx are initialized
            plotCanvas = document.getElementById('plot-canvas');
            plotCtx = plotCanvas ? plotCanvas.getContext('2d') : null;
            drawPlot();
        }
    };
function updateCurrentAngles() {
    if (!solution) {
        currentAnglesDiv.innerHTML = '';
        return;
    }
    const outAngle = interpolateOutputAngle(currentInputAngle);
    currentAnglesDiv.innerHTML = `<b>Current Position:</b> Input Angle: ${currentInputAngle.toFixed(1)}°, Output Angle: ${outAngle.toFixed(1)}°`;
}

function drawPlot() {
    if (!plotCtx || !solution) return;
    // Clear plot
    plotCtx.clearRect(0, 0, plotCanvas.width, plotCanvas.height);
    // Axes
    plotCtx.save();
    plotCtx.strokeStyle = '#333';
    plotCtx.lineWidth = 2;
    plotCtx.beginPath();
    plotCtx.moveTo(40, 20);
    plotCtx.lineTo(40, plotCanvas.height-40);
    plotCtx.lineTo(plotCanvas.width-20, plotCanvas.height-40);
    plotCtx.stroke();
    // Labels
    plotCtx.font = '14px Arial';
    plotCtx.fillStyle = '#333';
    plotCtx.fillText('Output Angle (deg)', 50, 30);
    plotCtx.save();
    plotCtx.translate(10, plotCanvas.height/2);
    plotCtx.rotate(-Math.PI/2);
    plotCtx.fillText('Input Angle (deg)', 0, 0);
    plotCtx.restore();
    // Range
    const minIn = Math.min(...inputAngles);
    const maxIn = Math.max(...inputAngles);
    const minOut = Math.min(...outputAngles);
    const maxOut = Math.max(...outputAngles);
    // Plot curve
    plotCtx.strokeStyle = '#1976d2';
    plotCtx.lineWidth = 2;
    plotCtx.beginPath();
    for (let a = minIn; a <= maxIn; a += 1) {
        const outA = interpolateOutputAngle(a);
        const x = 40 + ((a-minIn)/(maxIn-minIn))*(plotCanvas.width-60);
        const y = plotCanvas.height-40 - ((outA-minOut)/(maxOut-minOut))*(plotCanvas.height-60);
        if (a === minIn) plotCtx.moveTo(x, y);
        else plotCtx.lineTo(x, y);
    }
    plotCtx.stroke();
    // Mark specified task positions
    for (let i = 0; i < inputAngles.length; i++) {
        const a = inputAngles[i];
        const outA = outputAngles[i];
        const x = 40 + ((a-minIn)/(maxIn-minIn))*(plotCanvas.width-60);
        const y = plotCanvas.height-40 - ((outA-minOut)/(maxOut-minOut))*(plotCanvas.height-60);
        plotCtx.beginPath();
        plotCtx.arc(x, y, 6, 0, 2*Math.PI);
        plotCtx.fillStyle = '#d32f2f';
        plotCtx.fill();
        plotCtx.strokeStyle = '#333';
        plotCtx.stroke();
    }
    // Mark current position
    const currA = currentInputAngle;
    const currOutA = interpolateOutputAngle(currA);
    const currX = 40 + ((currA-minIn)/(maxIn-minIn))*(plotCanvas.width-60);
    const currY = plotCanvas.height-40 - ((currOutA-minOut)/(maxOut-minOut))*(plotCanvas.height-60);
    plotCtx.beginPath();
    plotCtx.arc(currX, currY, 8, 0, 2*Math.PI);
    plotCtx.fillStyle = '#388e3c';
    plotCtx.fill();
    plotCtx.strokeStyle = '#333';
    plotCtx.stroke();
    plotCtx.restore();
}
    controlsDiv.appendChild(slider);
}

addSlider();
drawSpecifiedPositions();
