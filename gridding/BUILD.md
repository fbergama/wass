# wassgridsurface Python package build instructions

NOTE: For developers only. For WASS users just install with

```
python -m pip install wassgridsurface
```

## Building

```
python -m pip install build
python -m build
```

package will be created in ./dist/


## Installing

```
python -m pip install ./dist/wassgridsurface-0.5.3.tar.gz
```

## Upload via twine

Upload to Test PyPi first:

```
twine upload -r testpypi dist/*
```

then, if everything looks good:

```
twine upload dist/*
```
