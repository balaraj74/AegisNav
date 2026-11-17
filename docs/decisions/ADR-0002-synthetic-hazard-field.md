# ADR-0002: Use an explicit synthetic hazard model for the first research prototype

- **Status:** Accepted for the competition starter; reassess before research publication.
- **Context:** The near-term target is repeatable proof of dynamic route selection, not validating physical combustion or gas dispersion. A CFD-grade model would add large setup and parameter uncertainty.
- **Decision:** Use a deterministic grid diffusion abstraction with configurable spread, decay, wind bias and persistent sources. Label it synthetic in code, UI, documentation and metrics.
- **Consequences:** Cheap deterministic scenarios and easy ablations. Results can only support algorithm behavior under the specified model; they do not validate real-world hazard prediction.
- **Future work:** Compare against a documented physical or domain-appropriate hazard simulator if a stronger scientific claim is needed.
