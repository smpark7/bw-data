flow_velocity=112.75 # cm/s
pre_flow_velocity=112.75
ini_temp=973     # initial temp
diri_temp=973    # dirichlet BC temp

[GlobalParams]
  temperature = temp
  integrate_p_by_parts = true
  gravity = '0 0 0'
  pspg = true
  supg = true
  alpha = 1
[]

[Mesh]
  file = 'msfr-ins.e'
[../]

[Problem]
  type = FEProblem
  coord_type = RZ
  rz_coord_axis = Y
[]

[Variables]
  [./ux]
    family = LAGRANGE
    order = FIRST
  [../]
  [./uy]
    family = LAGRANGE
    order = FIRST
  [../]
  [./p]
    family = LAGRANGE
    order = FIRST
  [../]
[]

[Kernels]
  [./mass]
    type = INSMassRZ
    variable = p
    u = ux
    v = uy
    p = p
    # pspg = true
    # alpha = '.333'
  [../]
  # [./x_time_deriv]
  #   type = INSMomentumTimeDerivative
  #   variable = ux
  # [../]
  # [./y_time_deriv]
  #   type = INSMomentumTimeDerivative
  #   variable = uy
  # [../]
  [./x_momentum_space]
    type = INSMomentumTractionFormRZ
    variable = ux
    u = ux
    v = uy
    p = p
    component = 0
    # supg = true
    # alpha = 1
  [../]
  [./y_momentum_space]
    type = INSMomentumTractionFormRZ
    variable = uy
    u = ux
    v = uy
    p = p
    component = 1
    # supg = true
    # alpha = 1
  [../]
  # [./buoyancy_x]
  #   type = INSBoussinesqBodyForce
  #   variable = ux
  #   block = 'blanket'
  #   dT = deltaT
  #   component = 0
  #   temperature = temp
  # [../]
  # [./buoyancy_y]
  #   type = INSBoussinesqBodyForce
  #   variable = uy
  #   block = 'blanket'
  #   dT = deltaT
  #   component = 1
  #   temperature = temp
  # [../]
[]

[Materials]
  [./fuel]
    type = GenericConstantMaterial
    prop_names = 'cp rho k mu'
    prop_values = '1 4.125e-3 1 .4'
    # prop_values = '1355 4.125e-3 .010097 1.0146e-4'
    block = 'fuel'
  [../]
  # [./rho_fuel]
  #   type = DerivativeParsedMaterial
  #   f_name = rho
  #   function = '(4983.56 - .882 * temp) * .000001'    # kg cm-3
  #   args = 'temp'
  #   derivative_order = 1
  #   block = 'fuel'
  # [../]
  # [./k_fuel]
  #   type = ParsedMaterial
  #   f_name = k
  #   function = '(0.928 + 8.397e-5 * temp) * .01'    # J cm-1 K-1
  #   args = 'temp'
  #   block = 'fuel'
  # [../]
  # [./mu_fuel]
  #   type = ParsedMaterial
  #   f_name = mu
  #   function = 'rho * exp(3689 / temp) * 5.55e-8 * 10000'
  #   material_property_names = 'rho'
  #   args = 'temp'
  #   block = 'fuel'
  # [../]
[]

[Functions]
  [./velFunc]
    type = ParsedFunction
    # value = '-4 * (y / 18.75 - (y / 18.75)^2) * (1 - exp(-t/20))'
    value = '-508.5025 * 4 * (y / 18.75 - (y / 18.75)^2) * (1 - exp(-t/7))'
  [../]
[]

[BCs]
  # Boussinesq
  [./ux_dirichlet]
    type = DirichletBC
    boundary = 'outer bottom top axial'
    variable = ux
    value = 0
  [../]
  [./uy_dirichlet]
    type = DirichletBC
    boundary = 'outer inlet top bottom'
    variable = uy
    value = 0
  [../]
  [./ux_inlet]
    type = FunctionDirichletBC
    boundary = 'inlet'
    variable = ux
    function = 'velFunc'
  [../]
  # [./uy_outlet]
  #   type = NeumannBC
  #   boundary = "top"
  #   variable = uy
  #   value = 0
  # [../]
[]

[Executioner]
  type = Transient
  end_time = 100

 nl_rel_tol = 1e-6
 nl_abs_tol = 1e-6

  solve_type = 'NEWTON'
  petsc_options = '-snes_converged_reason -ksp_converged_reason -snes_linesearch_monitor'
  petsc_options_iname = '-pc_type -pc_factor_shift_type'
  petsc_options_value = 'lu       NONZERO'
  line_search = 'none'
  # petsc_options_iname = '-pc_type -pc_asm_overlap -sub_pc_type -sub_pc_factor_levels'
  # petsc_options_value = 'asm      2               ilu          2'

  nl_max_its = 10
  l_max_its = 50

  dtmin = 1e-6
  dtmax = 3
  # dt = 1e-3
  [./TimeStepper]
    type = IterationAdaptiveDT
    dt = 1e-4
    cutback_factor = .4
    growth_factor = 1.2
    optimal_iterations = 20
    iteration_window = 4
  [../]
[]

[Preconditioning]
  [./FDP]
    type = SMP
    full = true
    solve_type = 'NEWTON'
  [../]
[]

[Postprocessors]
[]

[Outputs]
  perf_graph = true
  csv = true
  [./exodus]
    type = Exodus
    execute_on = 'timestep_end'
  [../]
[]

[Debug]
  show_var_residual_norms = true
[]

[ICs]
  [./ux_ic]
    type = ConstantIC
    variable = ux
    value = 0
  [../]
  [./uy_ic]
    type = ConstantIC
    variable = uy
    value = 0
  [../]
[]
