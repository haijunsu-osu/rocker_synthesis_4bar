/**
 * This module implements the analytical three‑position synthesis for a planar four‑bar
 * mechanism.  Given three precision pairs of input crank angles (θ) and output rocker
 * angles (φ) along with the ground link length r₁, it computes the remaining link
 * lengths r₂, r₃ and r₄.  The derivation follows the lecture notes “Lecture 12 –
 * Three position synthesis” from ME 3751/ME 562.  It starts from the standard
 * input–output equation of a four bar, normalises by the ground length and reduces
 * it to a linear equation in the three unknowns z₁, z₂ and z₃ (where z₂ = 1/r₂,
 * z₃ = 1/r₄ and z₁ = (1 + r₂² + r₄² – r₃²)/(2r₂r₄)).  Solving a 3×3 linear system
 * for these z variables yields the link ratios; r₂ and r₄ are the reciprocals of
 * z₂ and z₃, respectively, and r₃ is recovered from z₁ using the relationship
 * r₃² = 1 + r₂² + r₄² – 2r₂r₄z₁【619787799879262†screenshot】.  Finally, scaling by the specified ground link
 * length r₁ gives the actual physical lengths R₂, R₃ and R₄.
 */

// Helper: convert degrees to radians
function deg2rad(deg) {
  return (deg * Math.PI) / 180;
}

/**
 * Solve a 3×3 linear system Ax = b using Cramer's rule.  The input matrix A and
 * vector b must be arrays of numbers.  Returns an array [x0, x1, x2].  Throws
 * if the determinant is zero.
 *
 * @param {number[][]} A  3×3 matrix
 * @param {number[]} b    length‑3 vector
 * @returns {number[]}    solution vector x
 */
function solve3x3(A, b) {
  // Compute the determinant of A
  const detA =
    A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (Math.abs(detA) < 1e-12) {
    throw new Error('Matrix is singular or nearly singular');
  }
  // Helper to compute determinant of matrix replacing column col with b
  function detReplace(col) {
    const M = [
      [...A[0]],
      [...A[1]],
      [...A[2]],
    ];
    for (let i = 0; i < 3; i++) {
      M[i][col] = b[i];
    }
    return (
      M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
      M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
      M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])
    );
  }
  const det1 = detReplace(0);
  const det2 = detReplace(1);
  const det3 = detReplace(2);
  return [det1 / detA, det2 / detA, det3 / detA];
}

/**
 * Perform three‑position synthesis for a four‑bar mechanism.
 *
 * @param {number[]} thetaDeg  Array of three input crank angles (θ₁, θ₂, θ₃) in degrees
 * @param {number[]} phiDeg    Array of three output rocker angles (φ₁, φ₂, φ₃) in degrees
 * @param {number}   r1        Ground link length (base) in desired units
 * @returns {{r1: number, r2: number, r3: number, r4: number}}  Object with computed link lengths
 */
function threePositionSynthesis(thetaDeg, phiDeg, r1 = 1) {
  if (!Array.isArray(thetaDeg) || thetaDeg.length !== 3 || !Array.isArray(phiDeg) || phiDeg.length !== 3) {
    throw new Error('thetaDeg and phiDeg must be arrays of three numbers');
  }
  // Convert all angles to radians
  const theta = thetaDeg.map(deg2rad);
  const phi = phiDeg.map(deg2rad);
  // Build the matrix A and right‑hand side b from the design equations
  const A = [];
  const b = [];
  for (let i = 0; i < 3; i++) {
    A.push([1, Math.cos(phi[i]), -Math.cos(theta[i])]);
    b.push(Math.cos(phi[i] - theta[i]));
  }
  // Solve for z1, z2, z3
  const [z1, z2, z3] = solve3x3(A, b);
  // Recover the link ratios (r2, r4, r3) assuming r1 = 1
  const r2Scaled = 1 / z2;
  const r4Scaled = 1 / z3;
  const r3Squared = 1 + r2Scaled * r2Scaled + r4Scaled * r4Scaled - 2 * r2Scaled * r4Scaled * z1;
  const r3Scaled = Math.sqrt(Math.max(0, r3Squared));
  // Scale by the specified ground link length to get actual lengths
  const result = {
    r1: r1,
    r2: r2Scaled * r1,
    r3: r3Scaled * r1,
    r4: r4Scaled * r1,
  };
  return result;
}

// Export CommonJS interface when in a Node.js environment
if (typeof module !== 'undefined' && typeof module.exports !== 'undefined') {
  module.exports = {
    threePositionSynthesis,
    // Expose the low‑level solver for unit testing
    solve3x3,
  };
}

/* -------------------------------------------------------------------------
 * Position analysis of a four‑bar mechanism
 *
 * Given link lengths r1 (ground), r2 (input), r3 (coupler) and r4 (output) and
 * an input crank angle θ (in radians), this function solves for the output
 * rocker angle φ using the law of cosines.  Because a four‑bar can have two
 * assembly modes (open and crossed), the optional argument prevPhi allows
 * continuity: the returned φ is chosen to be the solution closest to prevPhi.
 * If prevPhi is undefined, the “open” solution (positive gamma) is returned.
 *
 * Returns φ in radians.
 */
function solveFourBarPhi(r1, r2, r3, r4, theta, prevPhi, mode) {
  // Position of A: end of input link
  const xA = r2 * Math.cos(theta);
  const yA = r2 * Math.sin(theta);
  // Distance d between A and the right ground pivot (O4)
  const dx = xA - r1;
  const dy = yA;
  const d = Math.hypot(dx, dy);
  // If the triangle cannot be formed, return NaN
  if (d > r3 + r4 || d < Math.abs(r3 - r4)) {
    return NaN;
  }
  // Angle between vector O4->A and horizontal axis
  const delta1 = Math.atan2(dy, dx);
  // Use law of cosines to find angle gamma at pivot O4
  // cos(gamma) = (d^2 + r4^2 - r3^2) / (2 d r4)
  let cosGamma = (d * d + r4 * r4 - r3 * r3) / (2 * d * r4);
  // Clamp due to numerical errors
  cosGamma = Math.min(1, Math.max(-1, cosGamma));
  const gamma = Math.acos(cosGamma);
  // Two possible solutions for phi
  const phi1 = delta1 + gamma;
  const phi2 = delta1 - gamma;
  // If a mode is explicitly provided, use it to choose the branch
  if (mode === 'open') return phi1;
  if (mode === 'closed' || mode === 'crossed') return phi2;
  // Otherwise choose based on previous phi for continuity
  if (prevPhi === undefined || Number.isNaN(prevPhi)) {
    // Default to open chain
    return phi1;
  }
  // Choose the solution closest to previous phi
  const diff1 = Math.abs(normalizeAngle(phi1 - prevPhi));
  const diff2 = Math.abs(normalizeAngle(phi2 - prevPhi));
  return diff1 < diff2 ? phi1 : phi2;
}

/**
 * Normalize an angle to the range [–π, π].
 * @param {number} angle  Angle in radians
 */
function normalizeAngle(angle) {
  let a = angle;
  while (a <= -Math.PI) a += 2 * Math.PI;
  while (a > Math.PI) a -= 2 * Math.PI;
  return a;
}

/**
 * Compute joint positions of the four‑bar mechanism for a given input angle.
 * Returns an object with the positions {O2, A, B, O4}, each as {x, y}, and
 * the corresponding rocker angle phi.
 *
 * @param {number} r1  ground link length
 * @param {number} r2  input link length
 * @param {number} r3  coupler link length
 * @param {number} r4  output link length
 * @param {number} theta  input angle in radians
 * @param {number|undefined} prevPhi  optional previous phi angle for continuity
 */
function computeFourBarPositions(r1, r2, r3, r4, theta, prevPhi, mode) {
  const O2 = { x: 0, y: 0 };
  const O4 = { x: r1, y: 0 };
  // Input joint position
  const A = { x: r2 * Math.cos(theta), y: r2 * Math.sin(theta) };
  // Solve for output angle
  const phi = solveFourBarPhi(r1, r2, r3, r4, theta, prevPhi, mode);
  // Output joint position
  const B = { x: O4.x + r4 * Math.cos(phi), y: O4.y + r4 * Math.sin(phi) };
  return { O2, A, B, O4, phi };
}

// Expose functions to the browser when loaded via <script>
if (typeof window !== 'undefined') {
  window.threePositionSynthesis = threePositionSynthesis;
  window.solveFourBarPhi = solveFourBarPhi;
  window.computeFourBarPositions = computeFourBarPositions;

  /**
   * Set up the user interface for the three‑position synthesis web application.
   * This function runs in the browser after DOM content is loaded and
   * attaches event handlers to the input fields, buttons, slider, and
   * canvases.  It draws the initial design positions, enables panning
   * and zooming, animates the mechanism, and plots the φ–θ curve for
   * the current linkage and assembly mode.  All variables local to
   * the interface are encapsulated within this function to avoid
   * polluting the global namespace.
   */
  function setupUI() {
    // Helper conversions
    const rad2deg = (rad) => (rad * 180) / Math.PI;

    // DOM elements
    const r1Input = document.getElementById('r1');
    const thetaInputs = [
      document.getElementById('theta1'),
      document.getElementById('theta2'),
      document.getElementById('theta3'),
    ];
    const phiInputs = [
      document.getElementById('phi1'),
      document.getElementById('phi2'),
      document.getElementById('phi3'),
    ];
    const synthesizeBtn = document.getElementById('synthesizeBtn');
    const canvas = document.getElementById('fourbarCanvas');
    const ctx = canvas.getContext('2d');
    const plotCanvas = document.getElementById('plotCanvas');
    const plotCtx = plotCanvas ? plotCanvas.getContext('2d') : null;
    const slider = document.getElementById('slider');
    const playPauseBtn = document.getElementById('playPauseBtn');
    const modeOpenRadio = document.getElementById('modeOpen');
    const modeClosedRadio = document.getElementById('modeClosed');

    // State variables
    let r1, r2, r3, r4;
    let thetaVals = [];
    let phiVals = [];
    let prevPhi;
    let assemblyMode = 'open';
    let playing = false;
    let animationId = null;
    // View state for panning/zooming
    let viewScale = 1;
    let offsetX = 0;
    let offsetY = 0;
    let isDragging = false;
    let lastMouseX = 0;
    let lastMouseY = 0;

    /**
     * Draw a light gray grid and axes on the mechanism canvas.  The
     * spacing of the grid lines scales with the current zoom level so
     * that the density appears constant.  The axes are drawn through
     * the centre of the canvas (adjusted by pan offsets).
     */
    function drawGrid() {
      const w = canvas.width;
      const h = canvas.height;
      ctx.save();
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#e0e0e0';
      ctx.fillRect(0, 0, w, h);
      const baseSpacing = 20;
      const spacing = baseSpacing * viewScale;
      ctx.strokeStyle = '#cccccc';
      ctx.lineWidth = 1;
      for (let x = offsetX % spacing; x <= w; x += spacing) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, h);
        ctx.stroke();
      }
      for (let y = offsetY % spacing; y <= h; y += spacing) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
      }
      // Axes
      ctx.strokeStyle = '#888888';
      ctx.beginPath();
      ctx.moveTo(w / 2 + offsetX, 0);
      ctx.lineTo(w / 2 + offsetX, h);
      ctx.moveTo(0, h / 2 + offsetY);
      ctx.lineTo(w, h / 2 + offsetY);
      ctx.stroke();
      ctx.restore();
    }

    /**
     * Convert a point in mechanism coordinates (origin at O2, x right, y up)
     * to canvas pixel coordinates, accounting for pan, zoom and ground
     * length scaling.
     */
    function worldToCanvas(pt) {
      const margin = 40;
      // Determine base scale such that 2*r1 fits the smaller dimension
      const baseScale = (Math.min(canvas.width, canvas.height) - 2 * margin) / (2 * r1);
      const scale = baseScale * viewScale;
      const cx = canvas.width / 2 + offsetX;
      const cy = canvas.height / 2 + offsetY;
      return {
        x: cx + pt.x * scale,
        y: cy - pt.y * scale,
      };
    }

    /**
     * Draw a single linkage configuration (r2, r3, r4) on the mechanism
     * canvas.  The positions object must contain O2, A, B, O4.  The
     * linkage is drawn in the specified colour.
     */
    function drawLinkage(pos, colour = 'blue') {
      const { O2, A, B, O4 } = pos;
      const pO2 = worldToCanvas(O2);
      const pA = worldToCanvas(A);
      const pB = worldToCanvas(B);
      const pO4 = worldToCanvas(O4);
      ctx.strokeStyle = colour;
      ctx.lineWidth = 2;
      ctx.beginPath();
      // r2
      ctx.moveTo(pO2.x, pO2.y);
      ctx.lineTo(pA.x, pA.y);
      // r3
      ctx.lineTo(pB.x, pB.y);
      // r4
      ctx.lineTo(pO4.x, pO4.y);
      ctx.stroke();
      // ground link
      ctx.beginPath();
      ctx.moveTo(pO2.x, pO2.y);
      ctx.lineTo(pO4.x, pO4.y);
      ctx.stroke();
    }

    /**
     * Draw the design linkage at each of the three specified precision
     * positions using distinct colours.  Also annotate each position
     * with its θ and φ values near the coupler joint for clarity.
     */
    function drawSpecifiedPositions() {
      const colours = ['red', 'green', 'purple'];
      for (let i = 0; i < 3; i++) {
        const theta = thetaVals[i];
        const phi = phiVals[i];
        const O2 = { x: 0, y: 0 };
        const O4p = { x: r1, y: 0 };
        const A = { x: r2 * Math.cos(theta), y: r2 * Math.sin(theta) };
        const B = { x: O4p.x + r4 * Math.cos(phi), y: O4p.y + r4 * Math.sin(phi) };
        drawLinkage({ O2: O2, A: A, B: B, O4: O4p }, colours[i]);
        // Draw label near coupler joint
        const label = `θ${i + 1}=${Math.round(rad2deg(theta) * 10) / 10}°, φ${i + 1}=${Math.round(rad2deg(phi) * 10) / 10}°`;
        const pB = worldToCanvas(B);
        ctx.fillStyle = colours[i];
        ctx.font = '12px Arial';
        ctx.fillText(label, pB.x + 5, pB.y - 5);
      }
    }

    /**
     * Map a slider value (0–1) to a θ value between θ1→θ2→θ3.  A
     * linear interpolation is used across the two segments, with the first
     * half of the slider covering θ₁→θ₂ and the second half covering
     * θ₂→θ₃.
     */
    function thetaFromSlider(t) {
      if (t <= 0.5) {
        const local = t / 0.5;
        return thetaVals[0] + local * (thetaVals[1] - thetaVals[0]);
      } else {
        const local = (t - 0.5) / 0.5;
        return thetaVals[1] + local * (thetaVals[2] - thetaVals[1]);
      }
    }

    /**
     * Animation loop callback.  Computes the current θ from the slider,
     * solves the mechanism for φ using the current assembly mode and
     * previous φ for continuity, draws the grid, the mechanism and the
     * design positions, and advances the slider if playing.
     */
    function animate() {
      const tVal = parseFloat(slider.value);
      const theta = thetaFromSlider(tVal);
      const pos = computeFourBarPositions(r1, r2, r3, r4, theta, prevPhi, assemblyMode);
      prevPhi = pos.phi;
      drawGrid();
      drawLinkage(pos, 'blue');
      drawSpecifiedPositions();
      // Draw φ–θ plot on the right, indicating the current (θ, φ)
      if (plotCtx) drawPhiThetaPlot(theta, pos.phi);
      if (playing) {
        let next = tVal + 0.002;
        if (next > 1) next = 0;
        slider.value = next.toFixed(3);
        animationId = requestAnimationFrame(animate);
      }
    }

    /**
     * Compute and draw the φ–θ relationship for the current linkage and
     * assembly mode on the secondary plot canvas.  The θ range is set
     * from θ₁ to θ₃, and the specified precision points are marked on
     * the plot.  The axes are scaled to the min/max θ and φ values.
     */
    function drawPhiThetaPlot(currentTheta, currentPhi) {
      const ctxp = plotCtx;
      const w = plotCanvas.width;
      const h = plotCanvas.height;
      const margin = 40;
      ctxp.clearRect(0, 0, w, h);
      ctxp.fillStyle = '#ffffff';
      ctxp.fillRect(0, 0, w, h);
      ctxp.strokeStyle = '#333333';
      ctxp.lineWidth = 1;
      // Determine θ and φ ranges
      const thetaMin = Math.min(...thetaVals);
      const thetaMax = Math.max(...thetaVals);
      // Compute φ values for a dense set of θ
      const samples = 100;
      let phiComputed = [];
      for (let i = 0; i <= samples; i++) {
        const t = i / samples;
        const theta = thetaMin + t * (thetaMax - thetaMin);
        const pos = computeFourBarPositions(r1, r2, r3, r4, theta, undefined, assemblyMode);
        phiComputed.push(pos.phi);
      }
      const phiMin = Math.min(...phiComputed.concat(phiVals));
      const phiMax = Math.max(...phiComputed.concat(phiVals));
      // Draw axes
      ctxp.beginPath();
      // X axis (θ)
      ctxp.moveTo(margin, h - margin);
      ctxp.lineTo(w - margin, h - margin);
      // Y axis (φ)
      ctxp.moveTo(margin, margin);
      ctxp.lineTo(margin, h - margin);
      ctxp.stroke();
      // Axis labels
      ctxp.font = '12px Arial';
      ctxp.fillStyle = '#000000';
      ctxp.fillText('θ (rad)', w - margin - 30, h - margin + 20);
      ctxp.save();
      ctxp.translate(margin - 20, margin + (h - 2 * margin) / 2);
      ctxp.rotate(-Math.PI / 2);
      ctxp.fillText('φ (rad)', 0, 0);
      ctxp.restore();
      // Plot computed φ–θ curve
      ctxp.strokeStyle = '#0077cc';
      ctxp.lineWidth = 2;
      ctxp.beginPath();
      for (let i = 0; i <= samples; i++) {
        const theta = thetaMin + (i / samples) * (thetaMax - thetaMin);
        const phi = phiComputed[i];
        const x = margin + ((theta - thetaMin) / (thetaMax - thetaMin)) * (w - 2 * margin);
        const y = margin + ((phiMax - phi) / (phiMax - phiMin)) * (h - 2 * margin);
        if (i === 0) ctxp.moveTo(x, y);
        else ctxp.lineTo(x, y);
      }
      ctxp.stroke();
      // Plot specified precision points
      ctxp.fillStyle = '#ff0000';
      for (let i = 0; i < 3; i++) {
        const theta = thetaVals[i];
        const phi = phiVals[i];
        const x = margin + ((theta - thetaMin) / (thetaMax - thetaMin)) * (w - 2 * margin);
        const y = margin + ((phiMax - phi) / (phiMax - phiMin)) * (h - 2 * margin);
        ctxp.beginPath();
        ctxp.arc(x, y, 4, 0, 2 * Math.PI);
        ctxp.fill();
      }
      // If current θ and φ are provided, draw a vertical line and a marker
      if (typeof currentTheta === 'number' && typeof currentPhi === 'number') {
        const cx = margin + ((currentTheta - thetaMin) / (thetaMax - thetaMin)) * (w - 2 * margin);
        const cy = margin + ((phiMax - currentPhi) / (phiMax - phiMin)) * (h - 2 * margin);
        ctxp.strokeStyle = '#888888';
        ctxp.lineWidth = 1;
        ctxp.beginPath();
        ctxp.moveTo(cx, margin);
        ctxp.lineTo(cx, h - margin);
        ctxp.stroke();
        ctxp.fillStyle = '#0000ff';
        ctxp.beginPath();
        ctxp.arc(cx, cy, 4, 0, 2 * Math.PI);
        ctxp.fill();
        // Annotate the current angles near the vertical line and marker
        const thetaDeg = rad2deg(currentTheta).toFixed(1);
        const phiDeg = rad2deg(currentPhi).toFixed(1);
        ctxp.fillStyle = '#333333';
        ctxp.font = '12px Arial';
        // Place θ label below the x-axis (slightly offset if necessary)
        ctxp.fillText(`θ=${thetaDeg}°`, cx - 20, h - margin + 15);
        // Place φ label near the marker, slightly above and to the right
        ctxp.fillText(`φ=${phiDeg}°`, cx + 5, cy - 5);
      }
    }

    /**
     * Perform synthesis based on current input field values, update
     * link lengths, enable controls, draw the design positions and
     * compute the φ–θ plot.  Called when the user clicks the
     * “Synthesize” button or on initial page load.
     */
    function performSynthesis() {
      r1 = parseFloat(r1Input.value);
      thetaVals = thetaInputs.map((inp) => deg2rad(parseFloat(inp.value)));
      phiVals = phiInputs.map((inp) => deg2rad(parseFloat(inp.value)));
      try {
        const result = threePositionSynthesis(
          thetaInputs.map((inp) => parseFloat(inp.value)),
          phiInputs.map((inp) => parseFloat(inp.value)),
          r1
        );
        r2 = result.r2;
        r3 = result.r3;
        r4 = result.r4;
        playPauseBtn.disabled = false;
        slider.disabled = false;
        slider.value = '0';
        playing = false;
        prevPhi = undefined;
        playPauseBtn.textContent = 'Play';
        drawGrid();
        drawSpecifiedPositions();
        if (plotCtx) drawPhiThetaPlot();
        // Immediately draw the initial mechanism configuration
        animate();
        // Print solution to console
        console.log('Synthesis result:', { r1: r1, r2: r2, r3: r3, r4: r4 });
      } catch (err) {
        alert('Error in synthesis: ' + err.message);
        console.error(err);
      }
    }

    // Event handlers
    playPauseBtn.addEventListener('click', () => {
      if (!playPauseBtn.disabled) {
        playing = !playing;
        playPauseBtn.textContent = playing ? 'Pause' : 'Play';
        if (playing) {
          prevPhi = undefined;
          animationId = requestAnimationFrame(animate);
        } else {
          cancelAnimationFrame(animationId);
        }
      }
    });
    slider.addEventListener('input', () => {
      if (!playing) {
        prevPhi = undefined;
        animate();
      }
    });
    synthesizeBtn.addEventListener('click', performSynthesis);
    modeOpenRadio.addEventListener('change', () => {
      if (modeOpenRadio.checked) {
        assemblyMode = 'open';
        prevPhi = undefined;
        if (!playing) {
          animate();
        }
      }
    });
    modeClosedRadio.addEventListener('change', () => {
      if (modeClosedRadio.checked) {
        assemblyMode = 'closed';
        prevPhi = undefined;
        if (!playing) {
          animate();
        }
      }
    });
    // Panning
    canvas.addEventListener('mousedown', (e) => {
      isDragging = true;
      lastMouseX = e.clientX;
      lastMouseY = e.clientY;
    });
    window.addEventListener('mouseup', () => {
      isDragging = false;
    });
    window.addEventListener('mousemove', (e) => {
      if (!isDragging) return;
      const dx = e.clientX - lastMouseX;
      const dy = e.clientY - lastMouseY;
      lastMouseX = e.clientX;
      lastMouseY = e.clientY;
      offsetX += dx;
      offsetY += dy;
      if (!playing) {
        if (r2 && r3 && r4) {
          const tVal = parseFloat(slider.value);
          const theta = thetaFromSlider(tVal);
          const pos = computeFourBarPositions(r1, r2, r3, r4, theta, prevPhi, assemblyMode);
          prevPhi = pos.phi;
          drawGrid();
          drawLinkage(pos, 'blue');
          drawSpecifiedPositions();
          if (plotCtx) drawPhiThetaPlot();
        } else {
          drawGrid();
        }
      }
    });
    // Zooming
    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;
      const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
      offsetX = mouseX - (mouseX - offsetX) * zoomFactor;
      offsetY = mouseY - (mouseY - offsetY) * zoomFactor;
      viewScale *= zoomFactor;
      if (!playing) {
        if (r2 && r3 && r4) {
          const tVal = parseFloat(slider.value);
          const theta = thetaFromSlider(tVal);
          const pos = computeFourBarPositions(r1, r2, r3, r4, theta, prevPhi, assemblyMode);
          prevPhi = pos.phi;
          drawGrid();
          drawLinkage(pos, 'blue');
          drawSpecifiedPositions();
          if (plotCtx) drawPhiThetaPlot();
        } else {
          drawGrid();
        }
      }
    });
    // Initial synthesis
    performSynthesis();
  }

  // Attach setupUI to DOMContentLoaded so that the UI is initialised once
  // the document has been fully parsed.
  if (typeof document !== 'undefined') {
    window.addEventListener('DOMContentLoaded', setupUI);
  }
}