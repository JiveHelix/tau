from conans import ConanFile, CMake


class TauConan(ConanFile):
    name = "tau"
    version = "1.10.8"

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

    no_copy_source = True

    def build(self):
        cmake = CMake(self)
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
        self.requires("jive/[~1.1]")
        self.requires("fields/[~1.3]")
        self.requires("eigen/[~3.4]")
        self.requires("pex/[>=0.9.5 <0.10]")
        self.requires("fmt/[~10]")
        self.requires("nlohmann_json/[~3]")
