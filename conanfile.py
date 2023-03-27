from conans import ConanFile, CMake


class TauConan(ConanFile):
    name = "tau"
    version = "1.7.2"

    scm = {
        "type": "git",
        "url": "https://github.com/JiveHelix/tau.git",
        "revision": "auto",
        "submodule": "recursive"}

    # Optional metadata
    license = "MIT"
    author = "Jive Helix (jivehelix@gmail.com)"
    url = "https://github.com/JiveHelix/tau"
    description = "Utilities for numerical computing."
    topics = (
        "Matrix Algebra",
        "Numerical Computing",
        "Computer Graphics")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    generators = "cmake"

    options = {
        "CMAKE_TRY_COMPILE_TARGET_TYPE":
            ["EXECUTABLE", "STATIC_LIBRARY", None],
        "fPIC": [True, False]}

    default_options = {
        "CMAKE_TRY_COMPILE_TARGET_TYPE": None,
        "fPIC": False}

    no_copy_source = True

    def build(self):
        cmake = CMake(self)

        if (self.options.CMAKE_TRY_COMPILE_TARGET_TYPE):
            cmake.definitions["CMAKE_TRY_COMPILE_TARGET_TYPE"] = \
                self.options.CMAKE_TRY_COMPILE_TARGET_TYPE

        if (self.options.fPIC):
            cmake.definitions["fPIC"] = self.options.fPIC

        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["tau"]

    def build_requirements(self):
        self.test_requires("catch2/2.13.9")

    def requirements(self):
        self.requires("jive/[~1.0]")
        self.requires("fields/[~1]")
        self.requires("eigen/[~3.4]")
        self.requires("pex/[>=0.8.9]")
        self.requires("fmt/[~8]")
