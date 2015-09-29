if [ -e model/JAXON_JVRC ]; then
    rm -rf model/JAXON_JVRC
fi
\cp -r JAXON_JVRC model
\cp task_configs/*.cnoid model
