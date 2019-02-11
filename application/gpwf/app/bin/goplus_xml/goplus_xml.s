//;======================================================================================================
//        AREA    Resource, CODE, READONLY
//;======================================================================================================
//            EXPORT GOPLUS_XML_START
//            EXPORT GOPLUS_XML_END
.section .rodata
            .global GOPLUS_XML_START
            .global GOPLUS_XML_END
GOPLUS_XML_START:
        .incbin  "../application/gpwf/app/bin/goplus_xml/goplus.xml"
GOPLUS_XML_END:
                .end
