#ifndef UIMANAGERSERVICE_H
#define UIMANAGERSERVICE_H


namespace LIMoSim {

class UiManager;

namespace ui {

class UiManagerService
{
public:
    static UiManagerService* getInstance();

    UiManager* getUiManager();
    void setUiManager(UiManager* _uiManager);

    bool getUiDataInitialized();
    void setUiDataInitialized(bool _initialized);
private:
    UiManager* m_uiManager;
    bool m_initialized;
    bool m_uiDataInitialized;
    UiManagerService();
};

} // namespace ui
} // namespace LIMoSim

#endif // UIMANAGERSERVICE_H
