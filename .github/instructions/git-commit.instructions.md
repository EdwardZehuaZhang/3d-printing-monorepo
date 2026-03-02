---
applyTo: '**'
---
---
description: Generate conventional commit messages
globs:
alwaysApply: true
---

# Git Commit Messages

Upon completion of tasks, generate a concise git commit message using conventional commits format.

**Format:** `<type>(<scope>): <description>`

## Types

`feat` `fix` `docs` `style` `refactor` `perf` `test` `build` `ci` `chore` `revert`

## Guidelines

- Use appropriate conventional commit type
- Optional scope for area of change (e.g., auth, api, ui)
- Keep message clear and descriptive

## Examples

```
feat(auth): add user authentication middleware
fix(api): resolve null pointer in user query
docs(readme): update installation instructions
refactor(db): simplify query builder logic
```
