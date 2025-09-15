# STYLE GUIDE

Purpose: Keep our texts simple, clear, and consistent. Use this guide when writing or editing any content in this repository.

Audience: University students with basic programming and robotics knowledge.

Principles:
- Prefer simple English. Short sentences. One idea per sentence.
- Write for quick scanning: headings, short paragraphs, bullet lists.
- Be precise. Avoid vague words (some, many, very). Use numbers and units.
- Use active voice and direct instructions ("Do X", "Use Y").
- Keep sections short. Remove anything not needed to learn or do the task.

Structure and formatting:
- Headings: Use H1 for page title, H2/H3 for sections. Do not skip levels.
- Paragraphs: 1–4 short sentences each.
- Lists: Prefer bullets for steps, properties, and tips. Use numbered lists only for strict sequences.
- Emphasis: Use sparingly. Prefer clear wording over bold/italic.
- Links: Use descriptive text ("PID basics" not "click here").

Terminology and notation:
- Units: Use SI with symbols in square brackets, e.g., velocity [m/s], angle [rad]. Put units next to values: 0.5 m, 10 rad/s.
- Symbols and code-like terms: wrap in inline code, e.g., `v`, `ω`, `θ`, `x`, `y`, `dt`.
- Greek letters: use ASCII names only if needed in code; otherwise use the symbol (ω, θ). Be consistent within a page.
- Sign conventions: State them once early (e.g., CCW positive, x forward, y left) and follow consistently.
- Numbers: Prefer digits ("3" not "three") when they carry meaning.

Math and equations:
- Keep math light. Explain concepts in words first.
- If equations are needed, prefer image equations already used in this repo or short inline relations like `v = r·ω`.
- Explain each variable near its first use. Include units.

Code examples:
- Use fenced code blocks with language tags: ```python, ```c++, ```bash.
- Keep examples minimal but correct. Show the core idea. Avoid long boilerplate.
- Add one-line purpose comment at top if not obvious.
- Handle obvious edge cases in examples (e.g., `dt > 0`, clamping output).
- Use clear, lower_snake_case for Python, lowerCamelCase for Arduino/C++ where consistent with existing files.
- Prefer self-contained snippets over partial fragments when possible.

Images and figures:
- Use existing images when available. Path should be relative and stable, e.g., `../images/file.png`.
- Always set informative alt text that describes the figure, not the filename.
- Refer to figures in text briefly and explain why it matters.

Consistency rules (from navigation texts):
- Kinematics: define pose `(x, y, θ)` and velocities `v` [m/s], `ω` [rad/s] up front.
- Differential drive: state wheel radius `r` and wheel separation `L` once.
- Controllers: when describing PID/PD, list what each term does and give a short algorithm.
- Sensor guides: start with what the sensor measures and the units.

Style do’s:
- Do start sections with a short summary sentence.
- Do show a tiny example or pseudocode after the concept.
- Do add a short "Practical Tips" or "Troubleshooting" list where relevant.
- Do keep variable names consistent with earlier sections.

Style don’ts:
- Don’t use long paragraphs or unexplained jargon.
- Don’t mix different sign conventions on the same page.
- Don’t add decorative words that don’t help understanding.
- Don’t rely on heavy formatting or long tables—use lists.

File and naming:
- Filenames: lowercase with underscores, e.g., `1_differential_chassis.md`.
- Images: keep under `../images/` with clear names; update alt text when reusing.

Quick checklist before you commit:
- Purpose clear in the first 2–3 lines?
- Simple English and active voice?
- Headings and short paragraphs used?
- Units and symbols consistent and defined?
- Examples minimal, correct, with language tags?
- Images (if any) have helpful alt text?
- Removed unnecessary content?

This guide is short by design. If in doubt, choose the simpler wording and keep the reader’s task in mind.