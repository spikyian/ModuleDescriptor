# Git Hooks

This directory contains git hooks for aiding work in this repository.

## Setup

Git normally runs hooks in `$GIT_DIR/hooks`.
But `$GIT_DIR` is not version controlled.
Instead, we use a directory `git-hook`.
Run this command to tell Git to use these hooks.

```$ git config core.hooksPath git-hooks```

This must be run once, before the first commit. 

## pre-commit Hook

All generated module descriptor files are created with an updated `"timestamp"` element.
The pre-commit hook is used to roll back any files where the **only** change
is the `"timestamp"` element.