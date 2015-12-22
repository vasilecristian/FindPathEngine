include "../../premake/common/"


solution "FindPathEngine"
    addCommonConfig()
    
    
project "FindPathEngine"
    files
    {
        "../src/**.h",
		"../src/**.c",
		"../src/**.cpp",
        "../include/**.h",
		"../include/**.c",
		"../include/**.cpp",
    }
    includedirs
    {
        "../include/",
		"../src/",
		"../../ThreadPool/include/"
    }
    kind "StaticLib"
    targetdir("../lib/" .. GetPathFromPlatform())
    if (IsWin32()) then
        defines {"WIN32"}
    end
    
    if IsWin8StoreApp() then
        -- WinRT
        removeflags { 'StaticRuntime' }
        flags { 'DisableWinRT' }
    end

    if IsIos() then
		files
		{
			"../src/**.m",
			"../src/**.mm",
			"../include/**.m",
			"../include/**.mm",
		}
        defines {"OS_IPHONE"}
        buildoptions { "-std=c++11 -stdlib=libc++ -x objective-c++ -Wno-error" }
		-- kind "WindowedApp"
		-- files { GetPathFromPlatform() .. "/Info.plist" }
	else
		-- kind "ConsoleApp"
	end

	if IsXCode() then
        addCommonXcodeBuildSettings()	
        
        xcodebuildsettings 
        {
            ["ONLY_ACTIVE_ARCH"] = "NO"
        }
        
        xcodebuildresources
        {
            "../data/**",
        }

        files 
        {
            "../data/**",
        }
        defines {"OS_IPHONE"}
		buildoptions { "-std=c++11 -stdlib=libc++ -x objective-c++ -Wno-error" }
		
		-- kind "WindowedApp"
		-- files { GetPathFromPlatform() .. "/Info.plist" }
    end

	
					
-------------------------------------------------------------------------------
--
-------------------------------------------------------------------------------
project "test"

	files 
	{ 
		"../test/*.cpp"
	}

	includedirs 
	{ 
		"../include",
	}
				
	--removeflags "NoRTTI"

	if IsMac() or IsIos() then
		kind "WindowedApp"
		files { "xcode4/".._OPTIONS['arch'].."/Info.plist" }
	else
		kind "ConsoleApp"
	end

	uuid "D3D73ADE-5BCB-E64C-84CE-B4749E7DD493"
	
	addCommonXcodeBuildSettings()

	if IsXCode() then 

		xcodebuildsettings 
		{
			
		}
	end	
	
	if IsMac() or IsIos() then
		links { "Foundation.framework" }
	end 
		
	links {"FindPathEngine"}

	targetname( "test" )

	targetdir ("../build/" .. GetPathFromPlatform())

	
	libdirs 
	{ 
		"../lib/" .. GetPathFromPlatform(),
	
	}	

	configuration "Release"
					
	configuration "Debug"
						
-------------------------------------------------------------------------------
--
-------------------------------------------------------------------------------
