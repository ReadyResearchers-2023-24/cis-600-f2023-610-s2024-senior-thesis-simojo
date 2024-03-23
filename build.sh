#!/usr/bin/env sh

pandoc \
  --filter pandoc-xnos \
  --defaults pdf.yaml \
  --to latex \
  --metadata-file config.yaml \
  --lua-filter .filters/abstract-to-meta.lua \
  --lua-filter .filters/sec-refs-better.lua \
  --lua-filter .filters/pagebreak.lua \
  --template template/thesis.tex \
  --citeproc \
  --csl https://www.zotero.org/styles/journal-of-the-acm \
  --bibliography references.bib \
  --highlight-style zenburn \
  --number-sections
