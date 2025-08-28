
# **Three Rocker Position Synthesis of Planar 4-bar Linkages**

---

This project implements an analytical synthesis and kinematic analysis tool for a planar four‑bar linkage. Given three precision pairs of crank and rocker angles, the tool computes the remaining link lengths (input, coupler and output links) and visualises the resulting mechanism. It also solves the closure equation at arbitrary crank angles to animate the linkage and plots the relationship between the input and output angles.

## Demo

![Demo animation](Rocker_Synthesis_Demo.gif)

## Using the Web Application

Click [here to open the web application](https://haijunsu-osu.github.io/rocker_synthesis_4bar/fourbar_rocker_synthesis.html) in a modern browser. 

Enter the ground link length r₁ and three pairs of input (θᵢ) and output (φᵢ) angles in degrees. Default values are provided in the form.

Click Synthesize. The tool computes the link lengths, draws the three specified positions in red/green/purple, and shows the initial mechanism configuration in blue. The φ–θ plot on the right displays the relationship between the input and output angles; the red points mark the specified positions.

Use the radio buttons (Open or Closed) to choose the assembly mode. The mechanism and plot update immediately.

Click Play to animate the mechanism between the three positions. The slider controls the crank angle. A moving vertical line in the φ–θ plot indicates the current input angle, and a blue marker shows the corresponding output angle; the numeric values of θ and φ are displayed near the line.

You can pan and zoom the mechanism canvas using mouse drag and wheel. The grid density remains constant under zoom.

## Analytical Solution Overview

Four‑bar synthesis is based on the Grashof input–output constraint for a planar four‑bar linkage. Let the ground pivot spacing be r₁ and let the other links be r₂ (input crank), r₃ (coupler), and r₄ (output rocker). If the input crank makes an angle θ with the horizontal and the output rocker makes an angle φ, the vector loop closure condition can be expressed explicitly in x– and y–components. Locating the joints at O₂ = (0,0), O₄ = (r₁, 0), A = (r₂ cos θ, r₂ sin θ) and B = (r₁ + r₄ cos φ, r₄ sin φ), and letting the coupler AB of length r₃ have orientation β, the loop O₂ → A → B → O₄ → O₂ yields

r₂ cos θ + r₃ cos β = r₁ + r₄ cos φ,    (x‑components)
r₂ sin θ + r₃ sin β = r₄ sin φ.         (y‑components)


These two equations are equivalent to the compact complex form
r₂ (cos θ + i sin θ) + r₃ e^{iβ} = r₄ (cos φ + i sin φ) + r₁, but the component form makes it clearer how the geometry fits together. Eliminating the unknown angle β (for example by using the law of cosines on triangle ABO₄) yields the well‑known scalar constraint

r₃² = r₁² + r₂² + r₄² + 2 r₁ r₄ cos φ − 2 r₁ r₂ cos θ − 2 r₂ r₄ cos(θ − φ).

Dividing through by r₁² normalises the equation (set r₁ = 1 for
synthesis). The goal is to determine r₂, r₃, and r₄ such that
the mechanism passes through three prescribed input–output angle pairs
(θ₁, φ₁), (θ₂, φ₂), and (θ₃, φ₃).

### Transformation to a Linear System

The quadratic terms can be linearised by recognising that the unknowns always appear in symmetric forms. Rearrange the normalised equation to collect the product r₂ r₄ multiplied by cos(φ − θ) on one side:

2 r₂ r₄ cos(φ − θ) = 1 + r₂² + r₄² − r₃² − 2 r₂ cos θ + 2 r₄ cos φ.

To obtain a linear relationship, introduce three auxiliary variables

z₁ = (1 + r₂² + r₄² − r₃²) / (2 r₂ r₄),
z₂ = 1 / r₂,
z₃ = 1 / r₄.

The definition of z₁ reflects the symmetric combination of link lengths appearing with the unknown r₃²; dividing by 2 r₂ r₄ normalises the expression. Setting z₂ = 1/r₂ and z₃ = 1/r₄ makes the remaining terms linear in cos θ and cos φ. Substituting these definitions and dividing the previous equation by 2 r₂ r₄ gives the constraint equation in linear form:

cos(φ − θ) = z₁ + z₂ cos φ − z₃ cos θ.

### Solving for the Link Ratios

For each precision pair (θᵢ, φᵢ) you have

cos(φᵢ − θᵢ) = z₁ + z₂ cos φᵢ − z₃ cos θᵢ.


Write these three equations in matrix form A z = b with

A = [[1, cos φ₁, −cos θ₁],
     [1, cos φ₂, −cos θ₂],
     [1, cos φ₃, −cos θ₃]],
z = [z₁, z₂, z₃]^T,
b = [cos(φ₁ − θ₁), cos(φ₂ − θ₂), cos(φ₃ − θ₃)]^T.


Because the matrix is non‑singular for distinct angles, solve for z₁, z₂, z₃ using Cramer’s rule or any 3×3 linear solver. The reciprocal relationships then give the link length ratios:

r₂ = 1 / z₂ (input link);

r₄ = 1 / z₃ (output link);

r₃ = √(1 + r₂² + r₄² − 2 r₂ r₄ z₁) (coupler length).

These lengths are scaled by the specified ground length r₁ to obtain the physical dimensions R₂, R₃ and R₄.

### Summary of the Solution Steps

Set up the input–output constraint from the vector loop closure
and normalise it by r₁.

Define auxiliary variables z₁, z₂, z₃ so the equation becomes
linear in cos θ and cos φ.

Write three linear equations using the prescribed (θᵢ, φᵢ) pairs.

Solve the 3×3 linear system A z = b to find z₁, z₂, z₃.

Recover the link ratios using r₂ = 1/z₂, r₄ = 1/z₃ and
r₃ = √(1 + r₂² + r₄² − 2 r₂ r₄ z₁).

Multiply by the ground length r₁ to obtain the actual link
lengths.

## Files

- `synthesis.js` – Library that provides the analytical synthesis and kinematic solvers. It exports the following functions:
  - `threePositionSynthesis(thetaDeg, phiDeg, r1)` – Performs three‑position synthesis based on the derivation in Lecture 12. It solves a 3×3 linear system for variables z1, z2 and z3 using Cramer’s rule, recovers the link length ratios, and scales them by the specified ground length r1 to obtain {r1, r2, r3, r4}.
  - `solveFourBarPhi(r1, r2, r3, r4, theta, prevPhi, mode)` – Solves the position analysis (closure equation) to find the rocker angle φ for a given crank angle θ. It uses the law of cosines and returns either the “open” or “closed” solution depending on the mode parameter, maintaining continuity with the optional prevPhi argument.
  - `computeFourBarPositions(r1, r2, r3, r4, theta, prevPhi, mode)` – Convenience wrapper that returns the joint positions {O2, A, B, O4} and the rocker angle for a given θ.

When loaded in a browser, `synthesis.js` also attaches a complete UI implementation. The script automatically initialises the page on DOMContentLoaded, reads input fields, draws the linkage and design positions, manages animation, provides panning/zooming, and plots the φ–θ curve. It requires no additional libraries.

- `fourbar.html` – A minimal HTML file that defines the user interface (input fields, buttons, canvases and radio buttons) and includes `synthesis.js`. All logic is contained in the script.

- `test_synthesis.js` – A Node test script that imports `threePositionSynthesis` and validates it against the example in the lecture notes. Run it with `node test_synthesis.js` to see the computed link lengths and verify correctness.

---
# References

- Lecture notes: [Lecture 12 Three position synthesis (PDF)](Lecture%2012%20Three%20position%20synthesis.pdf)
- "Kinematics, Dynamics, and Design of Machinery" by Waldron, Kinzel, and Agrawal. [Amazon link](https://www.amazon.com/Kinematics-Dynamics-Design-Machinery-Waldron/dp/1119723093)
---
## License

This project is provided for educational use in mechanism design and kinematic analysis courses. See the lecture notes referenced in the code comments for additional derivation details.

---

The file contains only plain Markdown syntax and will render correctly on GitHub without any additional processing.

