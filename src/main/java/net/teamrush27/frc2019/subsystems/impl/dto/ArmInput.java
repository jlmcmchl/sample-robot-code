package net.teamrush27.frc2019.subsystems.impl.dto;

public class ArmInput {

	private Double extensionInput;
	private Double rotationInput;
	
	
	public ArmInput(Double extensionInput, Double rotationInput) {
		this.extensionInput = extensionInput;
		this.rotationInput = rotationInput;
	}
	
	public Double getExtensionInput() {
		return extensionInput;
	}
	
	public void setExtensionInput(Double extensionInput) {
		this.extensionInput = extensionInput;
	}
	
	public Double getRotationInput() {
		return rotationInput;
	}
	
	public void setRotationInput(Double rotationInput) {
		this.rotationInput = rotationInput;
	}
}

