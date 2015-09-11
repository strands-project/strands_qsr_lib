## Generate documentation

From within the `docs` folder run the following:

To autogenerate the packages and modules documentation from their docstrings:
```bash
sphinx-apidoc ../src/ -o rst/ -e
```
*NOTE:* `sphinx-apidoc` does not overwrite any files, so if you want to regenerate the documentation for a package or a 
module then delete first its rst file in the `docs/rst` folder.

Then simply run:
```bash
make html
```

## `qsrs.rst`
`qsrs.rst` is generated from `qsrs.md` as writing tables in RsT is a pain. md files can be converted to rst ones using 
`pandoc`. The actual command run from within the `docs` folder is:

```bash
pandoc --from=markdown --to=rst --output=qsrs.rst qsrs.md
```

If pandoc converts "\`" to "\`\`" then do two string replacements:

1. "\`\`" to "\`"
2. "\` " to "\`   ", i.e 'backtilt+1 space' to 'backtilt+3 spaces'
