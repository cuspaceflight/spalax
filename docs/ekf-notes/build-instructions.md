# Building LaTeX documents

Although the PDF for the LaTeX documents in this directory are also included, it
is worth adding a note on how they're created. On my (@rjw57) machine, one can
build the PDFs via:
```console
$ xelatex ekf-notes && bibtex ekf-notes && xelatex ekf-notes && xelatex ekf-notes
```
The repeated calls to ``xelatex`` being necessary due to the interesting way TeX
attempts to require only one pass over the file when typesetting.

