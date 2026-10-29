# Publishing checklist

The repository is private and nothing has been published. On the day:

1. Run `tools/check_public_text.sh` and read `cargo package -p navmesh-rs --list`; the package
   must contain only `src/`, the README, the changelog, the licences and `NOTICE`.
2. Re-run the parity comparison (`docs/VERIFICATION.md`) on the final source, both ref widths,
   and update its results table if anything changed.
3. Re-run the release fuzz on the final source (rule: corpus seeds; each target in `fuzz/` runs
   until coverage has been flat for 10 minutes, at least 10 and at most 120 CPU-minutes; zero
   crashes) and record the receipt.
4. Final review of the README, `docs/VERIFICATION.md`, the rustdoc, `NOTICE` and the licences.
5. In `.github/workflows/ci.yml` enable `push` and `pull_request` triggers; in `release.yml`
   enable the `push: tags: ['v*']` trigger.
6. Add the `CARGO_REGISTRY_TOKEN` repository secret (a crates.io token scoped to publishing
   `navmesh-rs`).
7. Move the `[Unreleased]` changelog entries under `## [0.1.0] - <date>`, check `version` in
   `Cargo.toml`, commit, and push the tag `v0.1.0`. The release workflow checks that the tag
   matches `Cargo.toml`, runs `cargo publish --dry-run`, publishes, and creates the GitHub
   release from the changelog section.
8. Make the GitHub repository public.
