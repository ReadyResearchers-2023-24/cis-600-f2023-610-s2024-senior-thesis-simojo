let
  shellname = "thesis";
  nixpkgs = import (builtins.fetchGit {
    name = "nixos-unstable-2023-10-18";
    url = https://github.com/nixos/nixpkgs/;
    ref = "refs/heads/nixos-unstable";
    rev = "ca012a02bf8327be9e488546faecae5e05d7d749";
  }) {};

  ##########################################################
  ## patch for pandoc-xnos not working with pandoc v3.x.x ##
  ##########################################################

  pandoc-xnos-patch = nixpkgs.python311Packages.buildPythonPackage rec {
    name = "pandoc-xnos";
    src = builtins.fetchGit {
      url = https://github.com/nandokawka/pandoc-xnos/;
      rev = "284474574f51888be75603e7d1df667a0890504d";
    };
    doCheck = false; # skip tests, because the patch is hacky
    propagatedBuildInputs = [
      nixpkgs.python311Packages.pandocfilters
      nixpkgs.python311Packages.psutil
    ];
  };

  ###########################################################################################
  ## Make the packages part of the pandoc-xnos suite use pandoc-xnos-patch as a dependency ##
  ###########################################################################################

  pandoc-tablenos = nixpkgs.python311Packages.buildPythonPackage rec {
    pname = "pandoc-tablenos";
    version = "2.3.0";
    src = nixpkgs.python311Packages.fetchPypi {
      inherit pname version;
      sha256 = "sha256-I9CKGqyYGq++xpbrJl5ohCb5MQ9ugJdvN5stmytd4bw=";
    };
    doCheck = false;
    propagatedBuildInputs = [
      nixpkgs.python311Packages.pandocfilters
      pandoc-xnos-patch
    ];
  };
  pandoc-fignos = nixpkgs.python311Packages.buildPythonPackage rec {
    pname = "pandoc-fignos";
    version = "2.4.0";
    src = nixpkgs.python311Packages.fetchPypi {
      inherit pname version;
      sha256 = "sha256-cFjt6uSIJEEun+CQ3V9yUAqe99M6QDkYJrFzODl9iEk=";
    };
    doCheck = false;
    propagatedBuildInputs = [
      nixpkgs.python311Packages.pandocfilters
      pandoc-xnos-patch
    ];
  };
  pandoc-eqnos = nixpkgs.python311Packages.buildPythonPackage rec {
    pname = "pandoc-eqnos";
    version = "2.5.0";
    src = nixpkgs.python311Packages.fetchPypi {
      inherit pname version;
      sha256 = "sha256-8MySUHQzQs7yNMqyfgAGpSS/FV7K+Yl63wc5bTLalMc=";
    };
    doCheck = false;
    propagatedBuildInputs = [
      nixpkgs.python311Packages.pandocfilters
      pandoc-xnos-patch
    ];
  };
  # leaving out pandoc-secnos because of wacky build step in setup.py that checks if pandoc-secnos is on the path
  /* pandoc-secnos = nixpkgs.python311Packages.buildPythonPackage rec {
    pname = "pandoc-secnos";
    version = "2.2.2";
    src = nixpkgs.python311Packages.fetchPypi {
      inherit pname version;
      sha256 = "sha256-Frf+xMsHargABCyPs6QqReZUZR8CYdmVYIaOFX/6EHw=";
    };
    doCheck = false;
    propagatedBuildInputs = [
      nixpkgs.python311Packages.pandocfilters
      pandoc-xnos-patch
    ];
  }; */
in
  nixpkgs.mkShell {
    buildInputs = [
      nixpkgs.tectonic
      nixpkgs.pandoc
      nixpkgs.entr
      pandoc-xnos-patch
      pandoc-tablenos
      pandoc-fignos
      pandoc-eqnos
/*       pandoc-secnos */
      nixpkgs.python311
    ];
    shellHook = ''
      export NIX_SHELL_NAME=${shellname}
      alias pdf="pandoc --filter pandoc-xnos --defaults pdf.yaml --to latex --metadata-file config.yaml --lua-filter .filters/abstract-to-meta.lua --template template/thesis.tex --citeproc --csl https://www.zotero.org/styles/journal-of-the-acm --bibliography references.bib"
      alias pdflive="ls | entr pandoc --filter pandoc-xnos --defaults pdf.yaml --to latex --metadata-file config.yaml --lua-filter .filters/abstract-to-meta.lua --template template/thesis.tex --citeproc --csl https://www.zotero.org/styles/journal-of-the-acm --bibliography references.bib"
    '';
  }

# from https://github.com/tomduck/pandoc-tablenos: "Any use of `--filter pandoc-citeproc` or `--bibliography=FILE` should come *after* the `pandoc-tablenos` or `pandoc-xnos` calls.
