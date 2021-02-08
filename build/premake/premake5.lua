--
-- If premake command is not supplied an action (target compiler), exit!
--
-- Target of interest:
--     vs2019     (Visual Studio 2019)
--

-- we must have an ide/compiler specified
if (_ACTION == nil) then
  return
end

--
-- set include and library paths
--
COMMON_IncPath      = "../../"

local function createConsoleProject(project)
   project (project)
     kind("ConsoleApp")
     defines({"_CONSOLE"})
     location("../" .. _ACTION .. "/projects/%{prj.name}")
     targetname(project)
     targetdir("../../" .. project)
     includedirs({COMMON_IncPath})
     debugdir("../../" .. project)
     files({
        "../../" .. project .. "/**.h*",
        "../../" .. project .. "/**.c*"
     })
     --links({LibWindows})
end

workspace("physics-game-prg")

   location("../" .. _ACTION)

   -- configuration shared between all projects
   language("C++")
   characterset("MBCS")

   --
   -- Build (solution) configuration options:
   --     Release        (Application linked to Multi-threaded DLL)
   --     Debug          (Application linked to Multi-threaded Debug DLL)
   --
   configurations({ "Release", "Debug" })

   -- visual studio options and warnings
   -- /wd4351 (C4351 warning) - disable warning associated with array brace initialization
   -- /wd4996 (C4996 warning) - disable deprecated declarations
   -- /wd4005 (C4005 warning) - disable macro redefinition
   -- /wd4100 (C4100 warning) - disable unreferenced formal parameter
   -- /Oi - generate intrinsic functions
   disablewarnings({ "4351",  "4996", "4005", "4100" })
   buildoptions({ "/Oi" })

   -- common release configuration flags, symbols and libraries
   filter({ "configurations:Release" })
      optimize("On")
      -- favor speed over size
      buildoptions { "/Ot" }
      defines { "WIN32", "NDEBUG" }

   -- common debug configuration flags, symbols and libraries
   filter({ "configurations:Debug" })
      symbols("On")
      targetsuffix("_d")
      defines { "WIN32", "_DEBUG" }

   createConsoleProject("ch03-newtonian")
   createConsoleProject("ch04-kinematics")
   createConsoleProject("ch10-planes")


