from conans import ConanFile, CMake


class TauConan(ConanFile):
    name = "tau"
    version = "1.1.1"

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

    def package_id(self):
        self.info.header_only()

    def build_requirements(self):
        self.test_requires("catch2/2.13.8")

    def requirements(self):
        self.requires("jive/[~1.0]")
        self.requires("eigen/[~3.4]")
