

typedef enum InCommand
{
    CMD_NONE,               //test command
    CMD_GET_SETTINGS,       //������ ��������� �� ��
    CMD_SET_SETTINGS,       //�������� ���������
    CMD_SET_TIME,           //�������� ����� �����
    CMD_GET_AKKVOLTAGE,     //������ ���������� �� ������������
    CMD_SET_BLINK,          //������� ����������
    //CMD_WRITE_DATA,         //������ ������
    CMD_READ_DATA,          //������ ������
    CMD_SET_MODE_RUN,
    CMD_SET_MODE_CONN,
    CMD_SET_MODE_SLEEP,
    CMD_NEXT
}InCommand;

#pragma pack(push, 1)
typedef struct stCommand
{
    uint8_t signature;  //��������� ��������� 223
    uint8_t cmd;        //������� �� InCommand
    uint16_t lenbuf;    //������ ������ � �������
    uint8_t crc;
}stCommand;
#pragma pack(pop)

typedef enum OutAsk
{
    ASK_OK,
    ASK_ERROR,
    ASK_NEXT,
    ASK_MALLOC_ERROR,
    ASK_ERROR_CRC
}OutAsk;

//��������� �������
#define SIGNATURE_COMMAND                   223

typedef enum WORKMODE
{
    MODE_CONNECT,
    MODE_RUN,
    MODE_SLEEP
}WORKMODE;


#pragma pack(push, 1)
typedef struct SPORT_TAG_SETTINGS   //len=42
{
    WORKMODE    mode_tag;           //����� ������
    char        name_tag[20];       //��� �����
    int         timeut_conn;        //����� ����� ������� ����� ����� � ���. � ������ ����������
    int         timeut_run;         //����� ����� ������� ����� ����� � ���. � ������ ������
    uint8_t     powerble_tag;       //�������� BLE ����������� ����� �� 0 (-21 ��) �� 12 (+5 ��) �� ll.h ������ 404
    int8_t      treshold_tag;       //����� ���������������� �����
    char        password_tag[10];   //������ BLE

    //USER_SETTINGS        //len=103

    char        fam[20];        //�������
    char        imj[20];        //���
    char        otch[20];       //��������

    char        group[4];       //������
    char        razr[4];        //������
    uint16_t    godrojd;        //��� ��������
    char        colectiv[20];   //�������� ����������
    uint8_t     zabeg;          //����� ������
    uint16_t    startnum;       //��������� �����
    char        starttime[7];   //����� ������
    bool        lgota;          //���� �� ������
    bool        arenda;         //���� ��� ��� ������������
    uint8_t     signature;      //��������� SIGNATURE_EPROM_SETTINGS
}SPORT_TAG_SETTINGS;
#pragma pack(pop)

// ����������������� ����� ��������� �������
#define START_STATION_NUM         240
// ����������������� ����� �������� �������
#define FINISH_STATION_NUM        245
// ����������������� ����� ������� ��������
#define CHECK_STATION_NUM         248
// ����������������� ����� ������� �������
#define CLEAR_STATION_NUM         249
