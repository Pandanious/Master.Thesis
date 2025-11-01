FONT ?= "Computer Modern Roman"

# Find all main.tex files
# foo/main.tex -> .build/foo.pdf
SOURCES = $(shell find -name "main.tex")
_S0 = $(subst ./,,$(SOURCES))
_S1 = $(subst /main.tex,.pdf,$(_S0))
OUTPUTS=$(addprefix .build/,$(_S1))

# Find all .dot files
# src/bar.dot -> .build/dot/bar.pdf
DOT_FILES=$(wildcard src/*.dot)
DOTPDF_SRC=$(subst .dot,.pdf,$(DOT_FILES))
OUTPUTS_DOT=${subst src,.build/dot,${DOTPDF_SRC}}

.PHONY: all clean echo dot
all: dot $(OUTPUTS)
dot: $(OUTPUTS_DOT)

.build: 
	mkdir -p .build

.build/dot:
	mkdir -p .build/dot

# Rule for graphviz .dot files to .pdf
.build/dot/%.pdf: src/%.dot | .build/dot
	dot -Tpdf -Nfontname=${FONT} -Gfontname=${FONT} -Efontname=${FONT} $< -o $@

clean:
	rm -rf .build

# Generate rules for a folder
# openout_any=a for bug with 'latexmk -cd' with makeindex (2020)
# https://github.com/James-Yu/LaTeX-Workshop/issues/1932#issuecomment-583258858
define latex_rules
main_file := $(1)

## All files in the folder are dependencies
_SCARP0 := $$(subst /main.tex,,$$(main_file))
DEPS := $$(shell find $$(_SCARP0) ! -type d ! -path "*/build/*")

## foo/main.tex -> build/foo.pdf
_SCRAP1 := $$(addprefix .build/,$$(main_file))
OUTPUT := $$(subst /main.tex,.pdf,$$(_SCRAP1))

## Generates rule for output (e.g. .build/foo.pdf)
$$(OUTPUT): $$(main_file) $$(DEPS) | .build
	@echo "\033[1;33mBuilding $$@\033[0m"
	@NAME=`basename "$$@" | cut -d'.' -f1` && openout_any=a latexmk -pdf -interaction=nonstopmode -jobname=$$$$NAME -cd $$< -outdir=../.build 
endef

$(foreach source, $(_S0), $(eval $(call latex_rules, $(source))))

