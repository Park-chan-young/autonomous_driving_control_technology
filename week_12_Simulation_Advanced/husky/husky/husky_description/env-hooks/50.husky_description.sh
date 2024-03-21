if [[ ! -f "${HUSKY_DESCRIPTION}" ]]; then
    export HUSKY_DESCRIPTION=$(catkin_find --without-underlays --first-only husky_description urdf/description.xacro 2>/dev/null)
fi
