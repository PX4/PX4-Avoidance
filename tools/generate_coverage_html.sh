#!/bin/bash

bash tools/generate_coverage.sh

genhtml repo_total.info --output-directory coverage_html --branch-coverage
x-www-browser coverage_html/index.html
