## Generate documentation

From within the `docs` folder run the following:

To autogenerate the packages and modules documentation from their docstrings:
```bash
sphinx-apidoc ../qsr_lib/src/ -o rsts/api/ -e
```
*NOTE:* `sphinx-apidoc` does not overwrite any files, so if you want to regenerate the documentation for a package or a 
module then delete first its rst file in the `docs/rsts/api` folder.

Then simply run:
```bash
make html
```

## `qsrs.rst`
The table in `qsrs.rst` is generated from `qsrs_table.md` as writing tables in RsT is a pain.
md files can be converted to rst ones using  `pandoc`. 
The actual command run from within the `docs/rsts/handwritten/qsrs` folder is:

```bash
pandoc --from=markdown --to=rst --output=qsrs_table.rst qsrs_table.md
```

If pandoc converts "\`" to "\`\`" then do two string replacements:

1. "\`\`" to "\`"
2. "\`  " to "\`     ", i.e 'backtilt+2 spaces' to 'backtilt+6 spaces'
