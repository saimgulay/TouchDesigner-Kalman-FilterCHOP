import numpy as np
import td
from collections import deque

def dat_to_matrix(dat, dim):
    """
    Read a square DAT (dim×dim) into a NumPy array.
    Falls back to identity if missing or malformed.
    """
    try:
        if dat and dat.numRows == dim and dat.numCols == dim:
            M = np.zeros((dim, dim))
            for i in range(dim):
                for j in range(dim):
                    try:
                        M[i, j] = float(dat[i, j].val)
                    except:
                        M[i, j] = 0.0
            return M
    except:
        pass
    return np.eye(dim)


class KalmanFilter:
    """
    n-dimensional Kalman filter with robust inversion fallback.
    xₖ = A xₖ₋₁ + w, w~N(0,Q)
    zₖ = H xₖ   + v, v~N(0,R)
    """
    def __init__(self, dim, A, H, Q, R, x0, P0):
        self.dim = dim
        self.A = A; self.H = H
        self.Q = Q; self.R = R
        self.x = x0.copy()
        self.P = P0.copy()

    def predict(self):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)
        K = self.P @ self.H.T @ S_inv
        self.x += K @ y
        I = np.eye(self.dim)
        self.P = (I - K @ self.H) @ self.P

    def step(self, z):
        self.predict()
        self.update(z)
        return self.x


def onSetupParameters(scriptOp):
    page = scriptOp.appendCustomPage('KalmanFilter')

    # Existing parameters
    p = page.appendFloat('Processnoise',    label='Process Noise (Q)')
    p.default, p.min, p.max = 0.001, 0.0, 100.0

    p = page.appendFloat('Measurementnoise', label='Measurement Noise (R)')
    p.default, p.min, p.max = 0.1, 0.0, 100.0

    p = page.appendFloat('Initialstate',    label='Initial State')
    p.default = 0.0

    p = page.appendFloat('Initialcovariance', label='Initial Covariance')
    p.default, p.min, p.max = 1.0, 1e-6, 1e3

    p = page.appendFloat('Dt',              label='Time Step (Δt)')
    p.default, p.min = 1.0, 1e-9

    p = page.appendOP('Amatrixdat',         label='State Matrix DAT')
    p.help = 'Optional N×N DAT for A; identity if empty.'

    p = page.appendOP('Hmatrixdat',         label='Measurement Matrix DAT')
    p.help = 'Optional N×N DAT for H; identity if empty.'

    p = page.appendInt('Historylength',     label='History Length')
    p.default, p.min, p.max = 120, 1, 10000

    # —— New parameters ——    
    p = page.appendInt('Passes',             label='Number of Passes')
    p.help = 'How many times to apply the filter per sample.'
    p.default, p.min, p.max = 1, 1, 10

    p = page.appendToggle('Filterpersample', label='Filter per Sample')
    p.help = 'If ON, filter each sample in a multi-sample channel; otherwise filter one sample per cook.'
    p.default = 0

    p = page.appendFloat('Cutoff',           label='Cutoff Frequency (Hz)')
    p.help = 'Lower cutoff reduces slow-speed jitter.'
    p.default, p.min, p.max = 1.0, 0.0, 100.0

    p = page.appendFloat('Speedcoeff',       label='Speed Coefficient')
    p.help = 'Increase to reduce lag at high speeds.'
    p.default, p.min, p.max = 1.0, 0.0, 100.0

    p = page.appendFloat('Slopecutoff',      label='Slope Cutoff Frequency (Hz)')
    p.help = 'Avoids high-derivative bursts from jitter.'
    p.default, p.min, p.max = 1.0, 0.0, 100.0
    # —— End new parameters ——

    p = page.appendPulse('Reset',           label='Reset Filter & History')
    p.help = 'Clear filter state and history buffer.'

    # Ensure full-history mode
    scriptOp.isTimeSlice = False
    scriptOp.storage.pop('filter', None)
    scriptOp.storage.pop('history', None)


def onPulse(par):
    if par.name == 'Reset':
        owner = par.owner
        owner.storage.pop('filter', None)
        owner.storage.pop('history', None)
        print(f"{owner.name}: Kalman filter and history reset.")
    return


def onCook(scriptOp):
    # Force full-history mode
    scriptOp.isTimeSlice = False

    inputChop = scriptOp.inputs[0] if scriptOp.inputs else None
    if not inputChop or inputChop.numChans == 0:
        scriptOp.clear()
        return

    Nch   = inputChop.numChans
    HistL = max(scriptOp.par.Historylength.eval(), 1)

    # Read & clamp UI parameters
    q    = max(scriptOp.par.Processnoise.eval(),     1e-9)
    r    = max(scriptOp.par.Measurementnoise.eval(), 1e-9)
    x0   = scriptOp.par.Initialstate.eval()
    p0   = scriptOp.par.Initialcovariance.eval()
    dt   = scriptOp.par.Dt.eval()
    passes = max(scriptOp.par.Passes.eval(), 1)
    perSample = bool(scriptOp.par.Filterpersample.eval())
    cutoff    = scriptOp.par.Cutoff.eval()
    speedcoef = scriptOp.par.Speedcoeff.eval()
    slopecut  = scriptOp.par.Slopecutoff.eval()

    A_dat = scriptOp.par.Amatrixdat.eval()
    H_dat = scriptOp.par.Hmatrixdat.eval()

    # Initialize filter & history if needed
    filt = scriptOp.storage.get('filter')
    if filt is None or filt.dim != Nch:
        A = dat_to_matrix(A_dat, Nch)
        H = dat_to_matrix(H_dat, Nch)
        Q = np.eye(Nch) * q * dt
        R = np.eye(Nch) * r * dt
        x0_vec = np.full(Nch, x0)
        P0     = np.eye(Nch) * p0
        filt = KalmanFilter(Nch, A=A, H=H, Q=Q, R=R, x0=x0_vec, P0=P0)
        scriptOp.storage['filter'] = filt
        # Initialize history
        scriptOp.storage['history'] = [deque([x0]*HistL, maxlen=HistL) for _ in range(Nch)]

    # Rebuild history if length changed
    history = scriptOp.storage['history']
    if history[0].maxlen != HistL:
        scriptOp.storage['history'] = [deque([x0]*HistL, maxlen=HistL) for _ in range(Nch)]
        history = scriptOp.storage['history']

    # Update Q/R in case sliders moved
    filt.Q = np.eye(Nch) * q * dt
    filt.R = np.eye(Nch) * r * dt

    # Process filtering
    for _ in range(passes):
        if perSample:
            # apply filter to each sample in current history buffer
            for idx in range(HistL):
                z_vec = np.array([history[ch][idx] for ch in range(Nch)])
                x_vec = filt.step(z_vec)
                for ch in range(Nch):
                    history[ch][idx] = x_vec[ch]
        else:
            # filter only the latest sample
            z = np.array([inputChop[ch][0] for ch in range(Nch)])
            if np.any(~np.isfinite(z)):
                filt.predict()
                x = filt.x
            else:
                x = filt.step(z)
            for ch in range(Nch):
                history[ch].append(x[ch])

    # Output full-history CHOP
    scriptOp.clear()
    scriptOp.numSamples = history[0].maxlen
    outChans = [scriptOp.appendChan(inputChop[ch].name) for ch in range(Nch)]
    for i in range(history[0].maxlen):
        for ch in range(Nch):
            outChans[ch][i] = history[ch][i]
