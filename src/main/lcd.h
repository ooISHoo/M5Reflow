
class M5ReflowLCD{
public:
    M5ReflowLCD();
    ~M5ReflowLCD();

    typedef struct{
        const char* menustr;
        const char* suffixstr;
        int lower;
        int upper;        
    }MenuRecoard;
    
    void drawStatus(const char *,const char *);

    void drawBottomGuide(const char *,const char *,const char *);

    void drawReflowBase();
    void drawReflowParam();

    void drawMenu(MenuRecoard* record,int* params,int recordCnt,int inputDiff);
    void resetMenu();
    void MenuSelectToggle();

    void drawBakingBase();
    void drawBakingParam();

    void drawSettingBase();
    void drawSettingParam();

    void setBrink(bool);

    struct{
        float m_ExternalTemp;
        float m_TargetTemp;
        float m_InternalTemp;
        float m_RampRate;
        int m_StageCount;
        int m_Stage;
        int m_ReflowTime;
        bool m_GraphBrink;
        int m_PowerLower;
        int m_PowerUpper;

        bool m_TempError;
    }m_ReflowParam;
    int m_ReflowProfileParams[4][8];
    int m_ReflowProfileIndex;

    int m_MenuParams[8];
    int m_MenuCursor;
    bool m_MenuSelected;
    bool m_isBrink;
private:
};

inline void M5ReflowLCD::setBrink(bool brink)
{
    m_isBrink = brink;
}