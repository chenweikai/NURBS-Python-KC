Tessellation
^^^^^^^^^^^^

The ``tessellate`` module provides tessellation algorithms for surfaces. The following example illustrates the usage
scenario of the tessellation algorithms with surfaces.

.. code-block:: python
    :linenos:

    from geomdl import NURBS
    from geomdl import tessellate

    # Create a surface instance
    surf = NURBS.Surface()

    # Set tessellation algorithm (you can use another algorithm)
    surf.tessellator = tessellate.TriangularTessellate()

    # Tessellate surface
    surf.tessellate()

NURBS-Python uses :py:class:`.TriangularTessellate` class for surface tessellation by default.

.. note::

    To get better results with the surface trimming, you need to use a relatively smaller evaluation delta or a bigger
    sample size value. Recommended evaluation delta is :math:`d = 0.01`.

Class Reference
===============

Abstract Tessellator
--------------------

.. autoclass:: geomdl.tessellate.AbstractTessellate
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

Triangular Tessellator
----------------------

.. autoclass:: geomdl.tessellate.TriangularTessellate
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

Trim Tessellator
----------------

.. versionadded:: 5.0

.. autoclass:: geomdl.tessellate.TrimTessellate
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

Quadrilateral Tessellator
-------------------------

.. versionadded:: 5.2

.. autoclass:: geomdl.tessellate.QuadTessellate
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

Function Reference
==================

.. autofunction:: geomdl.tessellate.make_triangle_mesh

.. autofunction:: geomdl.tessellate.polygon_triangulate

.. autofunction:: geomdl.tessellate.make_quad_mesh

Helper Functions
================

.. autofunction:: geomdl.tessellate.surface_tessellate

.. autofunction:: geomdl.tessellate.surface_trim_tessellate
