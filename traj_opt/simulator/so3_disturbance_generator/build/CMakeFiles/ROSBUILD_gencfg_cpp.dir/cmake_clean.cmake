FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/so3_disturbance_generator/disturbance_uiConfig.h"
  "../docs/disturbance_uiConfig.dox"
  "../docs/disturbance_uiConfig-usage.dox"
  "../src/so3_disturbance_generator/cfg/disturbance_uiConfig.py"
  "../docs/disturbance_uiConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
