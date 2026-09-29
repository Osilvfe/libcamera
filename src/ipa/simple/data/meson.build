# SPDX-License-Identifier: CC0-1.0

conf_files = files([
    'sc1320cs.yaml',
    'sc820cs.yaml',
    'uncalibrated.yaml',
])

# The install_dir must match the name from the IPAModuleInfo
install_data(conf_files,
             install_dir : ipa_data_dir / 'simple',
             install_tag : 'runtime')
