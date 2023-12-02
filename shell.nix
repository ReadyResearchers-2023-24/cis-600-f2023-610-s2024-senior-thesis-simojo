let
  nixpkgs = import (builtins.fetchGit {
    name = "nixos-unstable-2023-10-18";
    url = https://github.com/nixos/nixpkgs/;
    ref = "refs/heads/nixos-unstable";
    rev = "ca012a02bf8327be9e488546faecae5e05d7d749";
  }) {};
  shellname = "thesis";
in
  nixpkgs.mkShell {
    buildInputs = [
      nixpkgs.tectonic
      nixpkgs.pandoc
      nixpkgs.entr
      nixpkgs.python311Packages.pandoc-xnos
    ];
    shellHook = ''
      alias pdf="pandoc --filter pandoc-xnos --defaults pdf.yaml --to latex --metadata-file config.yaml --lua-filter .filters/abstract-to-meta.lua --template template/thesis.tex"
      alias pdflive="ls | entr pandoc --filter pandoc-xnos --defaults pdf.yaml --to latex --metadata-file config.yaml --lua-filter .filters/abstract-to-meta.lua --template template/thesis.tex"
    '';
  }

# from https://github.com/tomduck/pandoc-tablenos: "Any use of `--filter pandoc-citeproc` or `--bibliography=FILE` should come *after* the `pandoc-tablenos` or `pandoc-xnos` calls.
