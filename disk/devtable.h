// Размер одного полного банка в блоках
#define banksize 0x400000

struct devtable_t{
    char* name;       // имя устройства
    uint32_t doffset; // смещение от начала банка
    uint32_t usize;   // размер одного диска на карте (с учетом округления)
    uint32_t realsize;// реальный размер дискового контейнера
    uint32_t maxdev;  // максимальный номер устройства данного типа
};

// Описатель структуры дискового банка
struct devtable_t devtable[] = {
//   name  offset     usize realsize  ndev
//-----------------------------------------    
    {"RK", 0,         6144,    6144,   7},
    {"DK", 0,         6144,    6144,   7},
    {"DW", 0xc000,  131072,  131072,   0},
    {"DX", 0x2c000,   4096,    502,    1},
    {"MY", 0x2e000,   2048,    1600,   3},
    {"DB", 0x30000, 393216,   393216,  7},
    {"DM", 0x330000, 65536,    53790,  7},
    {0}
};   
