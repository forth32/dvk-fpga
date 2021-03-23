all: font2mif fontextract fontlist fontreplace font.mif font-ksm.mif
font2mif: font2mif.c
fontextract: fontextract.c
fontlist: fontlist.c
fontreplace: fontreplace.c
font.mif: font-main.bin
	./font2mif font-main.bin font-main.mif
	./font2mif font-ksm.bin font-ksm.mif
font-main.mif: font-main.bin
	./font2mif font-main.bin font-main.mif
font-ksm.mif: font-ksm.bin
	./font2mif font-ksm.bin font-ksm.mif
	
