# mirte-python-api

This package provides the python API for the [MIRTE robot](https://mirte.org).
Please read the [MIRTE documentation](https://docs.mirte.org/develop/doc/api/mirte_python_api.html) 
for further documentation of this API.

## Install

```sh
pip install .
```

## Checks

To contribute to this repository the code needs to pass the python
[style check](https://github.com/mirte-robot/mirte-python/blob/develop/.github/workflows/python-check.yml)

To check this locally before you commit/push:
```sh
pip install black
black --check .
# Fix by using:
black .
```

