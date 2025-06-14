#!/bin/bash


# path
DIRECTORY_PKG=(
    "cr_bringup"
    "cr_bt_common"
    "cr_bt_pick_plave"
    "cr_controller"
    "cr_gui"
    "cr_hw_interface"
    "cr_interfaces"
    "cr_motion_core"
    "cr_remote_assistant"
    "cr_scene_management"
    "cr_vision"
)

# generate documentations
generate_docs() {
    for pkg_dir in "${DIRECTORY_PKG[@]}"; do
        doxyfile_path="$pkg_dir/Doxyfile"
        if [ -f "$doxyfile_path" ]; then
            echo "📘 Generating docs for $pkg_dir"
            pushd "$pkg_dir" > /dev/null
            mkdir -p docs
            doxygen - <<EOF
@INCLUDE             = Doxyfile
OUTPUT_DIRECTORY     = docs
EOF
            popd > /dev/null
        else
            echo "⛔ Warning: $doxyfile_path not found"
        fi
    done
}


#generate html readme
echo 3 | python3 html_conversion.py

generate_docs

#per far partire il server: 
#echo 2 | python3 html_conversion.py