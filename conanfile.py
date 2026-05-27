from conan import ConanFile

class MainProject(ConanFile):
    python_requires = "conan_template/[~6]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_runtime_measurement"
    description = "robotkernel runtime measurement module."
    exports_sources = ["*", "!.gitignore"]
    tool_requires = ["robotkernel_generator/[~6]@robotkernel/stable"]
    requires = ["robotkernel/[~6]@robotkernel/stable", "service_provider_process_data_inspection/[~6]@robotkernel/stable", ]
    
    def source(self):
        self.run(f"sed 's/AC_INIT(.*/AC_INIT([{self.name}], [{self.version}], [{self.author}])/' configure.ac.in > configure.ac")

