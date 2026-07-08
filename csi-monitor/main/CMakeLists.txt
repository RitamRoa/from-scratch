idf_component_register(
    SRCS "main.c" "csi_handler.c" "dsp_processor.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "../../heatmap3d.html" "join.html"
)

