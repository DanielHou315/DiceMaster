# Documentation Plan

## Overview

Auto-generate API reference docs from source code using rosdoc2 (official ROS2 documentation tool), hosted on ReadTheDocs (free tier, public repos).

## Tool: rosdoc2

- Wraps Sphinx + Doxygen + Breathe under the hood
- Supports Python (autodoc), C++ (Doxygen), and .msg/.srv definitions
- Accepts Markdown input via myst-parser
- Git hash/date embedding via sphinx-git extension
- Near-zero config for standard ROS2 package layouts

## Hosting: ReadTheDocs (free tier)

All repos are public, so the free tier works. Benefits over GitHub Pages:
- Auto-rebuilds on push (webhook, no Actions pipeline to maintain)
- Built-in versioning, search, and PDF export
- Custom domain support with free SSL

### Multi-repo strategy

**Option A (preferred):** Point ReadTheDocs at the parent `DiceMaster` repo with submodules. Builds a unified site from all packages.

**Option B:** Separate ReadTheDocs projects per submodule, linked together via intersphinx.

## What gets auto-generated

- `docs/api/` — API reference extracted from docstrings, function signatures, message definitions
- Includes git commit hash and build date in generated output

## What stays manual

- `docs/architecture.md`, `docs/setup.md`, `docs/developer_guide.md` — persistent docs that require human authorship
- These are included in the Sphinx build as static pages alongside the auto-generated API reference

## Setup (when ready)

1. Add `rosdoc2.yaml` to DiceMaster_Central root
2. Add `doc/conf.py` with Sphinx config (extensions: autodoc, breathe, myst-parser, sphinx-git)
3. Connect repo at readthedocs.org
4. Site publishes at `dicemaster.readthedocs.io` (or custom domain)
