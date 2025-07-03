from conan import ConanFile

class MainProject(ConanFile):
    python_requires = "conan_template/[^5.0.6]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_runtime_measurement"
    description = "robotkernel runtime measurement module."
    exports_sources = ["*", "!.gitignore"]
    requires = ["robotkernel/[~6]@robotkernel/unstable", "service_provider_process_data_inspection/[~6]@robotkernel/unstable", ]
