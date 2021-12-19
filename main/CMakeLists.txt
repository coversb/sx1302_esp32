idf_component_register(SRCS
                       "test_loragw_com_sx1250.c"
                       "src/loragw_com.c"
                       "src/loragw_spi.c"
                       "src/loragw_reg.c"
                       "src/loragw_sx1302.c"
                       "src/loragw_sx1250.c"
                       "src/loragw_aux.c"
                       "src/sx1250_com.c"
                       "src/sx1250_spi.c"

                       INCLUDE_DIRS
                       "."
                       "src")