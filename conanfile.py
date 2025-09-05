from conan import ConanFile


class TauConan(ConanFile):
    name = "tau"
    version = "1.12.0"

    python_requires = "boiler/0.1"
    python_requires_extend = "boiler.LibraryConanFile"

    license = "MIT"
    author = "Jive Helix (jivehelix@gmail.com)"
    url = "https://github.com/JiveHelix/tau"
    description = "Utilities for numerical computing."

    topics = (
        "Matrix Algebra",
        "Numerical Computing",
        "Computer Graphics")

    def build_requirements(self):
        self.test_requires("catch2/2.13.9")

    def requirements(self):
        self.requires("jive/[~1.4]", transitive_headers=True)
        self.requires("fields/[~1.5]", transitive_headers=True)
        self.requires("pex/[~1.1]", transitive_headers=True)
        self.requires("eigen/[~3.4]", transitive_headers=True)
        self.requires("fmt/[~10]", transitive_headers=True)
        self.requires("nlohmann_json/[~3]")
