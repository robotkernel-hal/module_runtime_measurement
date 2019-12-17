from conans import tools, python_requires

base = python_requires("conan_template/[~=5]@robotkernel/stable")

class MainProject(base.RobotkernelConanFile):
    name = "module_runtime_measurement"
    description = "robotkernel-5 runtime measurement module."
    exports_sources = ["*", "!.gitignore"] + ["!%s" % x for x in tools.Git().excluded_files()]
    requires = (
            "robotkernel/[~=5]@robotkernel/stable",
            "service_provider_process_data_inspection/[~=5]@robotkernel/stable" )

