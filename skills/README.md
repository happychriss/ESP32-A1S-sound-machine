# skills/

Reusable Claude skills for this project. Each file is a structured prompt loaded at session start to give Claude specialised procedures and conventions relevant to the project.

Skills cover areas such as:
- Session bootstrap and working conventions
- Build, flash, and debug workflows
- Component and dependency management
- Any other repeatable procedure worth encoding for reuse across sessions

Each file is self-describing via a YAML front-matter block (`name`, `description`) at the top. Add a new file here whenever a workflow or procedure is worth preserving beyond a single session.
