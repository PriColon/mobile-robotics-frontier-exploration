project = 'Autonomous Frontier Exploration'
author = 'Princess Colon, Rohit Mane, Manjunath Kondamu'
release = '1.0'

extensions = [
    'myst_parser',
    'sphinx.ext.autodoc',
]

html_theme = 'sphinx_rtd_theme'

html_theme_options = {
    'navigation_depth': 4,
    'collapse_navigation': False,
    'sticky_navigation': True,
}

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

html_static_path = ['_static']
html_css_files = ['custom.css']
html_js_files = ['js/lightbox.js']
