#include<WindowInterface.h>

#ifdef __cplusplus
extern "C" {
#endif
    __declspec(dllexport) DWORD NvOptimusEnablement = 1;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
#ifdef __cplusplus
}
#endif

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR    lpCmdLine,
    _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    if(!NodeWeft::WindowManager::init())
		return -1;
    // setup resource path
	NodeWeft::WindowManager::windowTitle = L"NodeWeft";
	NodeWeft::WindowManager::projectFileExtension = { L"NodeWeft Project File",L".nwproj" };    

    if (!NodeWeft::WindowInterface::run())
        return -1;
    return 0;
}