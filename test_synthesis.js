/*
 * Simple driver script to exercise the three‑position synthesis routine.  It
 * supplies the example from the lecture notes (θ₁ = 35.02°, θ₂ = 67.50°, θ₃ = 100.0°;
 * φ₁ = 91.21°, φ₂ = 101.79°, φ₃ = 117.19°; r₁ = 4.5 inches) and prints the
 * resulting link lengths.  This file can be run with `node test_synthesis.js`.
 */

const { threePositionSynthesis } = require('./synthesis');

// Input angles in degrees from the lecture problem【296687083953073†screenshot】
const thetaDeg = [35.02, 67.50, 100.0];
const phiDeg = [91.21, 101.79, 117.19];
const r1 = 4.5;

const result = threePositionSynthesis(thetaDeg, phiDeg, r1);

// Format the output with three decimal places
console.log('Computed link lengths for the three‑position synthesis example:');
console.log(`r1 = ${result.r1.toFixed(3)}`);
console.log(`r2 = ${result.r2.toFixed(4)}`);
console.log(`r3 = ${result.r3.toFixed(4)}`);
console.log(`r4 = ${result.r4.toFixed(4)}`);