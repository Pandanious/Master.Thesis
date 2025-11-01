# Latex-Documentation

All foo/main.tex are build using gitlab-ci.
Compiled PDFs are downloadable from: [gitlab](https://gitlab.ub.uni-bielefeld.de/CLF/projects/theses/ma-person-tracking/-/jobs/artifacts/master/browse?job=artifacts)

## Expose

Template for an Expose on a (Master)Thesis.

* Fill in your Information in metadata.tex.
* Add references to backmatter/bibliography.bib.
* Write content into mainmatter/*.tex files.

## Getting Started

### Prerequisites

(For Building locally)

* texlive (latex distribution)

  ```sh
  sudo apt install texlive-full
  ```

* graphviz (for .dot graphs)

  ```sh
  sudo apt install graphviz 
  ```

### Build

```sh
make all
```

