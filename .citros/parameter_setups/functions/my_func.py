def my_awesome_func(num):
    """
    A few things to notice:
    1. Any imports must be done inside the function.
    2. Return either native types or numpy scalars. Do not return non-scalar numpy values.
    """
    import numpy

    return numpy.multiply(num, 2.0)


def get_simulation_name(context):
    return context["simulation"]


def get_sid(context):
    return context["sid"]
