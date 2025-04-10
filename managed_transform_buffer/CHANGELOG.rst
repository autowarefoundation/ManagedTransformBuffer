^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package managed_transform_buffer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2025-04-10)
-----------
* fix: simplified get transform handling  (`#18 <https://github.com/autowarefoundation/ManagedTransformBuffer/issues/18>`_)
  * fix: sync for get_transfrom function redefinition
  * style: use camel case
  * style: remove redundant cast
  * style: snake case for logger name
  * style: destructor definition in header
  * style: remove unused ptr declaration
  * style: snake case for lambda function
  * style: keep provider as ref
  * fix: simplified get transform handling
  * style: improve error logging
  * fix: empty input cloud is not an error
  * fix: empty input cloud is not an error (2)
  ---------
* feat: assign requested stamp for static transforms (`#16 <https://github.com/autowarefoundation/ManagedTransformBuffer/issues/16>`_)
* perf: stop appending TF tree if already dynamic listener & fix logging (`#13 <https://github.com/autowarefoundation/ManagedTransformBuffer/issues/13>`_)
* feat: extend API with PCL point clouds & eigen matrix and affine transformation (`#12 <https://github.com/autowarefoundation/ManagedTransformBuffer/issues/12>`_)
* feat: remove unnecessary async calls (`#11 <https://github.com/autowarefoundation/ManagedTransformBuffer/issues/11>`_)
* feat: rework mutexes (`#10 <https://github.com/autowarefoundation/ManagedTransformBuffer/issues/10>`_)
* feat: const func & fix typos (`#9 <https://github.com/autowarefoundation/ManagedTransformBuffer/issues/9>`_)
* [pre-commit.ci] pre-commit autoupdate (`#8 <https://github.com/autowarefoundation/ManagedTransformBuffer/issues/8>`_)
  * [pre-commit.ci] pre-commit autoupdate
  updates:
  - [github.com/pre-commit/pre-commit-hooks: v4.6.0 → v5.0.0](https://github.com/pre-commit/pre-commit-hooks/compare/v4.6.0...v5.0.0)
  - [github.com/igorshubovych/markdownlint-cli: v0.41.0 → v0.44.0](https://github.com/igorshubovych/markdownlint-cli/compare/v0.41.0...v0.44.0)
  - [github.com/scop/pre-commit-shfmt: v3.9.0-1 → v3.10.0-2](https://github.com/scop/pre-commit-shfmt/compare/v3.9.0-1...v3.10.0-2)
  - [github.com/pycqa/isort: 5.13.2 → 6.0.1](https://github.com/pycqa/isort/compare/5.13.2...6.0.1)
  - [github.com/psf/black: 24.8.0 → 25.1.0](https://github.com/psf/black/compare/24.8.0...25.1.0)
  - [github.com/pre-commit/mirrors-clang-format: v18.1.8 → v19.1.7](https://github.com/pre-commit/mirrors-clang-format/compare/v18.1.8...v19.1.7)
  - [github.com/cpplint/cpplint: 1.6.1 → 2.0.0](https://github.com/cpplint/cpplint/compare/1.6.1...2.0.0)
  - [github.com/python-jsonschema/check-jsonschema: 0.29.2 → 0.31.2](https://github.com/python-jsonschema/check-jsonschema/compare/0.29.2...0.31.2)
  * style: add missing headers
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* feat: rework constructor, expand API, improve logging, fix sync (`#7 <https://github.com/autowarefoundation/ManagedTransformBuffer/issues/7>`_)
* perf: use single TF listener & add singleton (`#5 <https://github.com/autowarefoundation/ManagedTransformBuffer/issues/5>`_)
  * feat: reduce impl to single temp listener
  * feat: use singleton pattern
  * fix: typo
  ---------
* init commit
* Contributors: Amadeusz Szymko, pre-commit-ci[bot]
