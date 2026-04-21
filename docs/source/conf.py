# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
import os
import sys

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "MAVez"
copyright = "2025, The Pennsylvania State University Unmanned Aerial Systems Club"
author = "The Pennsylvania State University Unmanned Aerial Systems Club"
version = "3.10.0"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx_autodoc_typehints",
]

templates_path = ["_templates"]
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "furo"
html_static_path = ["_static"]

# Set the path to the source code directory
sys.path.insert(0, os.path.abspath("../../src"))

# -- Autodoc configuration --------------------------------------------------
autodoc_mock_imports = ["pymavlink"]
autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
}
