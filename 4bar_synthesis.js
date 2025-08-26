// 4bar_synthesis.js
// Interactive synthesis of planar 4-bar linkages

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
    // Draw input/output links for each specified position
    for (let i = 0; i < inputAngles.length; i++) {
        drawLinkage(groundLength, 60, 60, 80, inputAngles[i], outputAngles[i], true);
    }
}

function leastSquaresSolver(groundLen, inputAngles, outputAngles) {
    // Solve for inputLen, outputLen, couplerLen (min square error)
    // For each position, the coupler length = sqrt((Cx - Dx)^2 + (Cy - Dy)^2)
    // We want couplerLen to be the same for all positions
    // Set up Ax = b for least squares
    const n = inputAngles.length;
    const A = [];
    const b = [];
    for (let i = 0; i < n; i++) {
        const theta1 = inputAngles[i] * Math.PI / 180;
        const theta2 = outputAngles[i] * Math.PI / 180;
        // C = A + inputLen*[cos(theta1), sin(theta1)]
        // D = B + outputLen*[cos(theta2), sin(theta2)]
        // A = [-groundLen/2, 0], B = [groundLen/2, 0]
        // couplerLen^2 = (Cx - Dx)^2 + (Cy - Dy)^2
        // Expand: (inputLen*cos(theta1) + groundLen/2 - outputLen*cos(theta2) - groundLen/2)^2 + (inputLen*sin(theta1) - outputLen*sin(theta2))^2 = couplerLen^2
        // Let x = [inputLen, outputLen, couplerLen]
        // For each i: f_i(x) = sqrt((Cx - Dx)^2 + (Cy - Dy)^2) - couplerLen = 0
        // Linearize: (Cx - Dx)^2 + (Cy - Dy)^2 = couplerLen^2
        // So: (inputLen*cos(theta1) - outputLen*cos(theta2))^2 + (inputLen*sin(theta1) - outputLen*sin(theta2))^2 = couplerLen^2
        // Expand and collect terms:
        // inputLen^2 + outputLen^2 - 2*inputLen*outputLen*cos(theta1 - theta2) = couplerLen^2
        // So: inputLen^2 + outputLen^2 - 2*inputLen*outputLen*cos(theta1 - theta2) - couplerLen^2 = 0
        // This is nonlinear, but we can solve by least squares for inputLen, outputLen, couplerLen
        // We'll use a simple grid search for demonstration
    }
    // Simple grid search (for demo, not efficient)
    let best = null;
    let minErr = Infinity;
    for (let inputLen = 40; inputLen <= 200; inputLen += 5) {
        for (let outputLen = 40; outputLen <= 200; outputLen += 5) {
            for (let couplerLen = 40; couplerLen <= 200; couplerLen += 5) {
                let err = 0;
                for (let i = 0; i < n; i++) {
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

function synthesize() {
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
}
function interpolateOutputAngle(inputAngle) {
    // Linear interpolation for output angle
    if (inputAngles.length === 1) return outputAngles[0];
    if (inputAngle <= inputAngles[0]) return outputAngles[0];
    if (inputAngle >= inputAngles[inputAngles.length-1]) return outputAngles[outputAngles.length-1];
    for (let i = 1; i < inputAngles.length; i++) {
        if (inputAngle <= inputAngles[i]) {
            const t = (inputAngle - inputAngles[i-1]) / (inputAngles[i] - inputAngles[i-1]);
            return outputAngles[i-1] + t * (outputAngles[i] - outputAngles[i-1]);
        }
    }
    return outputAngles[outputAngles.length-1];
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
        }
    };
    controlsDiv.appendChild(slider);
}

addSlider();
drawSpecifiedPositions();
