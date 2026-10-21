/* Reentrant wrapper around the generated SkySurfer_Core (ERT Reusable function). See plant_core.h. */
#include "plant_core.h"

#include <math.h>
#include <string.h>

#include "SkySurfer_Core_private.h" /* generated; also defines the rtm access macros */

struct PlantCore {
  RT_MODEL_SkySurfer_Core_T model;
  DW_SkySurfer_Core_T dwork;
  X_SkySurfer_Core_T states;
  XDot_SkySurfer_Core_T derivatives;
  XDis_SkySurfer_Core_T disabled;
  ExtU_SkySurfer_Core_T input;
  ExtY_SkySurfer_Core_T output;
  PlantCoreInitial initial;
  uint64_t step_index;
  uint32_t ready;
  uint32_t diverged;
};

/* Point the model at this instance's own data. Mirrors the pointer setup of the generated
 * SkySurfer_Core_initialize(), so that a byte copy of an instance can be made self-consistent. */
static void bind(PlantCore *core) {
  RT_MODEL_SkySurfer_Core_T *m = &core->model;
  m->dwork = &core->dwork;
  m->contStates = &core->states;
  m->derivs = (double *)&core->derivatives;
  m->contStateDisabled = &core->disabled;
  rtsiSetSimTimeStepPtr(&m->solverInfo, &m->Timing.simTimeStep);
  rtsiSetTPtr(&m->solverInfo, &rtmGetTPtr(m));
  rtsiSetStepSizePtr(&m->solverInfo, &m->Timing.stepSize0);
  rtsiSetdXPtr(&m->solverInfo, &m->derivs);
  rtsiSetContStatesPtr(&m->solverInfo, (double **)&m->contStates);
  rtsiSetNumContStatesPtr(&m->solverInfo, &m->Sizes.numContStates);
  rtsiSetNumPeriodicContStatesPtr(&m->solverInfo, &m->Sizes.numPeriodicContStates);
  rtsiSetPeriodicContStateIndicesPtr(&m->solverInfo, &m->periodicContStateIndices);
  rtsiSetPeriodicContStateRangesPtr(&m->solverInfo, &m->periodicContStateRanges);
  rtsiSetContStateDisabledPtr(&m->solverInfo, (bool **)&m->contStateDisabled);
  rtsiSetErrorStatusPtr(&m->solverInfo, (&rtmGetErrorStatus(m)));
  rtsiSetRTModelPtr(&m->solverInfo, m);
  m->intgData.y = m->odeY;
  m->intgData.f[0] = m->odeF[0];
  m->intgData.f[1] = m->odeF[1];
  m->intgData.f[2] = m->odeF[2];
  m->intgData.f[3] = m->odeF[3];
  rtsiSetSolverData(&m->solverInfo, (void *)&m->intgData);
  rtmSetTPtr(m, &m->Timing.tArray[0]);
}

static void set_initial(PlantCore *core) {
  const PlantCoreInitial *ic = &core->initial;
  double *u = core->input.InitialConditions; /* [U0 V0 W0 H0 Phi0 Theta0 Psi0] */
  u[0] = ic->velocity_body_m_s[0];
  u[1] = ic->velocity_body_m_s[1];
  u[2] = ic->velocity_body_m_s[2];
  u[3] = ic->height_m;
  u[4] = ic->euler_deg[0];
  u[5] = ic->euler_deg[1];
  u[6] = ic->euler_deg[2];
}

uint32_t plant_core_api_version(void) { return PLANT_CORE_API_VERSION; }

size_t plant_core_size(void) { return sizeof(PlantCore); }

int32_t plant_core_init(PlantCore *core, const PlantCoreInitial *initial) {
  if (core == NULL || initial == NULL) return PLANT_CORE_ERR_INPUT;
  const double *values = (const double *)initial;
  for (size_t k = 0; k < sizeof(PlantCoreInitial) / sizeof(double); k++) {
    if (!isfinite(values[k])) return PLANT_CORE_ERR_INPUT;
  }
  memset(core, 0, sizeof(*core));
  core->initial = *initial;
  bind(core);
  SkySurfer_Core_initialize(&core->model, &core->input, &core->output);
  set_initial(core); /* initialize() clears the inputs */
  core->input.Wind[2] = 1.0; /* steady wind only, zero: the source model's setting */
  core->ready = 1;
  return PLANT_CORE_OK;
}

int32_t plant_core_step(PlantCore *core, const double controls[PLANT_CORE_CONTROL_COUNT],
                      double snapshot[PLANT_CORE_SNAPSHOT_COUNT]) {
  if (core == NULL || controls == NULL) return PLANT_CORE_ERR_INPUT;
  if (!core->ready || core->diverged) return PLANT_CORE_ERR_STATE;
  for (int k = 0; k < PLANT_CORE_CONTROL_COUNT; k++) {
    if (!isfinite(controls[k])) return PLANT_CORE_ERR_INPUT;
  }
  memcpy(core->input.Controls, controls, sizeof(core->input.Controls));
  SkySurfer_Core_step(&core->model, &core->input, &core->output);
  core->step_index++;
  for (int k = 0; k < PLANT_CORE_SNAPSHOT_COUNT; k++) {
    if (!isfinite(core->output.CoreSnapshot[k])) {
      core->diverged = 1;
      return PLANT_CORE_ERR_DIVERGED;
    }
  }
  if (snapshot != NULL) memcpy(snapshot, core->output.CoreSnapshot, sizeof(core->output.CoreSnapshot));
  return PLANT_CORE_OK;
}

int32_t plant_core_set_wind(PlantCore *core, const double wind[PLANT_CORE_WIND_COUNT]) {
  if (core == NULL || wind == NULL) return PLANT_CORE_ERR_INPUT;
  for (int k = 0; k < PLANT_CORE_WIND_COUNT; k++) {
    if (!isfinite(wind[k])) return PLANT_CORE_ERR_INPUT;
  }
  if (wind[0] < 0.0 || wind[3] < 0.0 || (wind[2] != 1.0 && wind[2] != 2.0)) return PLANT_CORE_ERR_INPUT;
  memcpy(core->input.Wind, wind, sizeof(core->input.Wind));
  return PLANT_CORE_OK;
}

void plant_core_default_wind_parameters(PlantCoreWindParameters *p) {
  if (p == NULL) return;
  memset(p, 0, sizeof(*p));
  /* Source model values (FW_SkySurfer_HIL.slx); amplitudes zero so that mode 2 has no gust unless asked. */
  p->gust_length_m[0] = 120.0;
  p->gust_length_m[1] = 120.0;
  p->gust_length_m[2] = 80.0;
  p->turbulence_seed[0] = 23341.0;
  p->turbulence_seed[1] = 23342.0;
  p->turbulence_seed[2] = 23343.0;
  p->turbulence_seed[3] = 23344.0;
}

int32_t plant_core_set_wind_parameters(const PlantCoreWindParameters *p) {
  if (p == NULL) return PLANT_CORE_ERR_INPUT;
  const double *values = (const double *)p;
  for (size_t k = 0; k < sizeof(*p) / sizeof(double); k++) {
    if (!isfinite(values[k])) return PLANT_CORE_ERR_INPUT;
  }
  for (int k = 0; k < 3; k++) {
    if (p->gust_length_m[k] <= 0.0) return PLANT_CORE_ERR_INPUT;
  }
  if (p->shear_speed_6m_m_s < 0.0) return PLANT_CORE_ERR_INPUT;
  for (int k = 0; k < 4; k++) {
    if (p->turbulence_seed[k] < 0.0 || p->turbulence_seed[k] != floor(p->turbulence_seed[k])) return PLANT_CORE_ERR_INPUT;
  }
  SkySurfer_Core_P.DiscreteWindGustModelOn_t_0 = p->gust_start_s;
  for (int k = 0; k < 3; k++) {
    SkySurfer_Core_P.DiscreteWindGustModelOn_d_m[k] = p->gust_length_m[k];
    SkySurfer_Core_P.DiscreteWindGustModelOn_v_m[k] = p->gust_magnitude_m_s[k];
  }
  SkySurfer_Core_P.Distanceintogustx_d_m = p->gust_length_m[0]; /* the integrator limits of the same block */
  SkySurfer_Core_P.Distanceintogusty_d_m = p->gust_length_m[1];
  SkySurfer_Core_P.Distanceintogustz_d_m = p->gust_length_m[2];
  SkySurfer_Core_P.WindShearModel_W_20 = p->shear_speed_6m_m_s;
  SkySurfer_Core_P.WindShearModel_Wdeg = p->shear_from_deg;
  SkySurfer_Core_P.DrydenWindTurbulenceModelContinuousqr_Wdeg = p->turbulence_from_deg;
  for (int k = 0; k < 4; k++) SkySurfer_Core_P.DrydenWindTurbulenceModelContinuousqr_Seed[k] = p->turbulence_seed[k];
  return PLANT_CORE_OK;
}

int32_t plant_core_copy(PlantCore *dst, const PlantCore *src) {
  if (dst == NULL || src == NULL) return PLANT_CORE_ERR_INPUT;
  if (dst == src) return PLANT_CORE_OK;
  memcpy(dst, src, sizeof(*dst));
  bind(dst);
  return PLANT_CORE_OK;
}

_Static_assert(sizeof(X_SkySurfer_Core_T) == PLANT_CORE_STATE_COUNT * sizeof(double), "continuous state count changed");

int32_t plant_core_get_state(const PlantCore *core, double x[PLANT_CORE_STATE_COUNT]) {
  if (core == NULL || x == NULL) return PLANT_CORE_ERR_INPUT;
  if (!core->ready) return PLANT_CORE_ERR_STATE;
  memcpy(x, &core->states, sizeof(core->states));
  return PLANT_CORE_OK;
}

int32_t plant_core_set_rigid_state(PlantCore *core, const double rigid[13]) {
  if (core == NULL || rigid == NULL) return PLANT_CORE_ERR_INPUT;
  if (!core->ready || core->diverged || core->step_index == 0) return PLANT_CORE_ERR_STATE;
  for (int k = 0; k < 13; k++) if (!isfinite(rigid[k])) return PLANT_CORE_ERR_INPUT;
  double norm = 0.0;
  for (int k = 6; k < 10; k++) norm += rigid[k] * rigid[k];
  if (fabs(norm - 1.0) > 1e-9) return PLANT_CORE_ERR_INPUT;
  memcpy(&core->states, rigid, 13 * sizeof(double));
  return PLANT_CORE_OK;
}

int32_t plant_core_eval(PlantCore *core, const double x[PLANT_CORE_STATE_COUNT],
                      const double controls[PLANT_CORE_CONTROL_COUNT], double xdot[PLANT_CORE_STATE_COUNT],
                      double snapshot[PLANT_CORE_SNAPSHOT_COUNT]) {
  if (core == NULL || x == NULL || xdot == NULL) return PLANT_CORE_ERR_INPUT;
  if (!core->ready) return PLANT_CORE_ERR_STATE;
  for (int k = 0; k < PLANT_CORE_STATE_COUNT; k++) {
    if (!isfinite(x[k])) return PLANT_CORE_ERR_INPUT;
  }
  memcpy(&core->states, x, sizeof(core->states));
  core->diverged = 0;
  int32_t code = plant_core_step(core, controls, snapshot);
  /* The generated ode4 stores f(t_k, x) in its first stage buffer before integrating. */
  memcpy(xdot, core->model.odeF[0], sizeof(double) * PLANT_CORE_STATE_COUNT);
  if (code != PLANT_CORE_OK) return code;
  for (int k = 0; k < PLANT_CORE_STATE_COUNT; k++) {
    if (!isfinite(xdot[k])) return PLANT_CORE_ERR_DIVERGED;
  }
  return PLANT_CORE_OK;
}

uint64_t plant_core_step_index(const PlantCore *core) { return core == NULL ? 0 : core->step_index; }

double plant_core_time_s(const PlantCore *core) {
  return core == NULL ? 0.0 : (double)core->step_index * PLANT_CORE_DT_S;
}
